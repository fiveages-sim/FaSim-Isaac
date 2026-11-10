"""在**正在运行编排**的 Isaac Sim 会话里旁路采集 Replicator 数据。

推荐用法
========
不要直接调本模块 —— 用交互式 CLI::

    ./scripts/sdg.sh capture      # 选机器人/任务/相机/输出目录/fps
    ./scripts/sdg.sh status       # 看进度
    ./scripts/sdg.sh stop         # 结束

CLI 通过 ``<runtime>/capture.sock`` 把指令发给随 Isaac 自动加载的扩展
（``fasim/replicator/sdg/scripts/extension.py``），本模块负责真正的采集。

也可以在 Script Editor 里直接调 ``arm_capture()``（调试用）。

路线
====
编排当主干（闭环、高成功率），Replicator 当工具（出相机数据）。
不额外起 ROS 通信、不接管控制 —— 只挂在同一个 Isaac 进程里"看"。

为什么能不影响编排
==================
ros2_control（``ROS=fiveages`` 变体）跑在 **Isaac 进程内**，和渲染循环
共享同一个 ``app.update()`` 时钟。本模块：

- **不自己开循环**，挂 ``physics step`` 事件回调；
- 回调里只 ``annotator.get_data()``（读已有缓冲） + ``np.savez``（1.3ms），
  不阻塞；
- 渲染由 Replicator tick 图跟着 ``app.update()`` 自动驱动。**不要调
  ``rep.orchestrator.step()``** —— Kit 内会抛 OrchestratorError。

产物::

    <out>/frames/frame_000123.npz   rgb / depth / joint_pos / sim_step
    <out>/session.json              相机参数、帧数、物理步数、错误数

注意：不要在 Script Editor 或任何回调里写 while / sleep 循环 —— 会饿死
Kit 主循环，物理停住、OCS2 卡死。
"""

from __future__ import annotations

import json
import sys
from pathlib import Path
from typing import Any

import numpy as np

#: 模块级句柄，供 stop_capture() 找到已挂载的会话
_S: dict[str, Any] = {
    "armed": False,
    "sub": None,
    "cameras": [],
    "out": None,
    "every": 10,
    "step": 0,
    "saved": 0,
    "skipped": 0,
    "t0": None,
    "records": [],
    "size": (640, 480),
}


def _log(msg: str) -> None:
    print(f"[capture] {msg}", flush=True)


def _code_stamp() -> str:
    """代码版本戳（文件 mtime），用来判断 Isaac 里跑的是不是最新代码。

    扩展 import 后模块缓存在 sys.modules，**改文件不会热重载**。
    如果采集结果和代码不符，先看这个时间戳是不是最新的。
    """
    try:
        import os
        import time

        mt = os.path.getmtime(__file__)
        return f"{time.strftime('%m-%d %H:%M:%S', time.localtime(mt))}"
    except Exception:  # noqa: BLE001
        return "unknown"


def _camera_specs(task, camera: str | None):
    """挑出要采的相机。``camera`` 为空则全部，否则按 name 过滤。"""
    specs = list(getattr(task, "cameras", None) or [])
    if not specs:
        return []
    if not camera:
        return specs
    want = {c.strip() for c in camera.split(",") if c.strip()}
    picked = [s for s in specs if s.name in want]
    if not picked:
        _log(f"WARN 找不到相机 {want}，可选: {[s.name for s in specs]}")
    return picked


def _to_np(x: Any) -> np.ndarray:
    """warp array / torch tensor / numpy → numpy 数组。"""
    if hasattr(x, "numpy"):
        return np.asarray(x.numpy(), dtype=np.float64)
    return np.asarray(x, dtype=np.float64)


def arm_capture(
    out_dir: str = "",
    every: int = 10,
    camera: str = "head",
    robot_key: str = "unitree_g1d",
    task_key: str = "sinopec_test_tube",
    width: int | None = None,
    height: int | None = None,
    record_joints: bool = True,
    depth: bool = True,
    name: str = "",
    record_ee: bool = True,
    record_objects: bool = True,
) -> dict | None:
    """挂载采集器并立即返回（非阻塞）。

    Args:
        out_dir: 输出目录。留空 → ``<data_root>/<robot>/<task>/<时间戳>/``，
            其中 ``data_root`` 由 ``FASIM_SDG_DATA`` 决定（默认 ``~/fasim_data``）。
        every: 每多少个**物理步**采一帧。177Hz 下 every=10 ≈ 17.7fps。
            渲染越频繁物理越慢；控制受影响就调大（20 / 40）。
        camera: 相机名，逗号分隔多个；空字符串=**不采相机**。
        robot_key / task_key: 选 ``configs/<robot>/`` 与 ``tasks/<task>.yaml``。
        width/height: 覆盖分辨率；None 用配置值。**320x240 可显著降开销**。
        record_joints: 记录关节角（action 标签）。
        depth: 采深度图。关掉能省一半磁盘。
        name: 会话子目录名；留空用时间戳。
        record_ee: 记录左右末端位姿（tcp，world 系），来自 robot.yaml 的 ee_prims。
        record_objects: 记录 tracked_prims 里的物体位姿（试管/架子等）。

    Returns:
        ``{"out":..., "cameras":[...], "every":..., "what":[...]}``；已挂载返回 None。
    """
    if _S["armed"]:
        _log("已挂载，先调用 stop_capture()")
        return None

    _log(f"live_capture 代码版本: {_code_stamp()}  (文件 mtime)")

    from fasim.replicator.sdg import runtime

    ext = runtime.ext_home()
    if str(ext) not in sys.path:
        sys.path.insert(0, str(ext))

    import carb
    import omni.physx
    import omni.replicator.core as rep
    import omni.usd

    from fasim.replicator.sdg.config_io import load_robot_config, load_task_config

    robot = load_robot_config(robot_key)
    task = load_task_config(robot_key, task_key)

    if str(out_dir or "").strip():
        out = Path(str(out_dir)).expanduser().resolve()
    else:
        out = runtime.default_session_dir(robot_key, task_key, name)
    (out / "frames").mkdir(parents=True, exist_ok=True)

    # 相机：camera 为空 = 不采相机（只采状态），此时不建 render product
    cams: list[dict] = []
    if str(camera or "").strip():
        specs = _camera_specs(task, camera)
        if not specs:
            _log("ERROR 指定的相机都找不到")
            return None
        for spec in specs:
            w = int(width or (spec.resolution[0] if spec.resolution else 640))
            h = int(height or (spec.resolution[1] if spec.resolution else 480))
            try:
                rp = rep.create.render_product(spec.prim_path, (w, h))
            except Exception as exc:  # noqa: BLE001
                _log(f"WARN 相机 {spec.name} 创建失败 ({spec.prim_path}): {exc}")
                continue
            rgb = rep.annotators.get("rgb")
            rgb.attach(rp)
            dep = None
            if depth:
                dep = rep.annotators.get("distance_to_image_plane")
                dep.attach(rp)
            cams.append({"name": spec.name, "rp": rp, "rgb": rgb, "depth": dep,
                         "size": (w, h), "prim": str(spec.prim_path)})
            _log(f"相机 {spec.name}: {w}x{h} @ {spec.prim_path}")
        if not cams:
            _log("ERROR 所有相机都创建失败")
            return None
    else:
        _log("不采相机（只采状态）")

    # 关节角（action 标签）。拿不到就跳过，不影响其他数据。
    art = None
    dof_names: list[str] = []
    if record_joints:
        try:
            from isaacsim.core.experimental.prims import Articulation

            art = Articulation(robot.articulation_prim)
            dof_names = list(art.dof_names)
            _log(f"关节标签: {len(dof_names)} 个 DOF")
        except Exception as exc:  # noqa: BLE001
            _log(f"WARN 关节读取不可用，跳过关节: {exc}")
            art = None

    # 末端位姿（tcp，world 系）—— 来自 robot.yaml 的 ee_prims。
    # 与层1 的 /left_current_pose 语义对齐：都是 tcp 的世界位姿。
    ee_views: dict[str, Any] = {}
    if record_ee:
        try:
            from isaacsim.core.experimental.prims import XformPrim

            for side, prim in (getattr(robot, "ee_prims", None) or {}).items():
                try:
                    ee_views[side] = XformPrim(str(prim))
                except Exception as exc:  # noqa: BLE001
                    _log(f"WARN 末端 {side} 不可用（{prim}）: {exc}")
            if ee_views:
                _log(f"末端位姿: {list(ee_views)}（world 系）")
        except Exception as exc:  # noqa: BLE001
            _log(f"WARN 末端位姿不可用: {exc}")

    # 交互物体位姿 —— 来自 task.yaml 的 tracked_prims
    obj_views: dict[str, Any] = {}
    obj_labels: list[str] = []
    if record_objects:
        try:
            from isaacsim.core.experimental.prims import RigidPrim

            for label, prim in (getattr(task, "tracked_prims", None) or {}).items():
                try:
                    obj_views[label] = RigidPrim(str(prim))
                    obj_labels.append(label)
                except Exception as exc:  # noqa: BLE001
                    _log(f"WARN 物体 {label} 不可用（{prim}）: {exc}")
            if obj_views:
                _log(f"物体位姿: {obj_labels}（world 系）")
        except Exception as exc:  # noqa: BLE001
            _log(f"WARN 物体位姿不可用: {exc}")

    what = []
    if cams:
        what.append("camera")
    if art is not None:
        what.append("joint")
    if ee_views:
        what.append("ee")
    if obj_views:
        what.append("object")
    if not what:
        _log("ERROR 什么都没采（相机/关节/末端/物体全部不可用）")
        return None

    import time

    _S.update(
        armed=True, cameras=cams, out=out, every=max(1, int(every)),
        step=0, saved=0, skipped=0, t0=time.monotonic(), records=[],
        size=cams[0]["size"] if cams else (0, 0),
        ee_views=ee_views, obj_views=obj_views, what=what,
        code_stamp=_code_stamp(),
    )

    def _on_physics_step(dt: float) -> None:
        """物理步进回调：按 every 降采样，取最近渲染的一帧落盘。必须快速返回。

        **不要调 ``rep.orchestrator.step()``** —— 在 Kit 进程内它会抛
        ``OrchestratorError: Synchronous call to step can only be performed in
        a standalone workflow and may not be made from within Kit``。
        （层2 能调是因为 ``SimulationApp(headless=True)`` 属于 standalone。）

        Kit 里渲染由 Replicator 的 tick 图跟着 ``app.update()`` 自动驱动，
        所以这里只要 ``get_data()`` 取最近一帧即可。实测 33/33 帧互不相同，
        不存在重复缓冲问题。
        """
        _S["step"] += 1
        if _S["step"] % _S["every"] != 0:
            _S["skipped"] += 1
            return
        try:
            idx = _S["saved"]
            payload: dict[str, Any] = {
                "frame": idx,
                "sim_step": _S["step"],
                "wall_time": time.monotonic() - _S["t0"],
            }
            for cam in _S["cameras"]:
                rgb = cam["rgb"].get_data()
                if rgb is not None:
                    payload[f"{cam['name']}_rgb"] = rgb
                if cam["depth"] is not None:
                    dep = cam["depth"].get_data()
                    if dep is not None:
                        payload[f"{cam['name']}_depth"] = dep
                payload[f"{cam['name']}_size"] = cam["size"]
                payload[f"{cam['name']}_prim"] = cam["prim"]
            if art is not None:
                try:
                    q = art.get_dof_positions()
                    payload["joint_pos"] = _to_np(q).astype(np.float32).reshape(1, -1)
                    payload["joint_names"] = dof_names
                except Exception:  # noqa: BLE001
                    pass
            # 末端位姿（world）：<side>_pos / <side>_quat
            for side, view in _S.get("ee_views", {}).items():
                try:
                    p, q = view.get_world_poses()
                    payload[f"{side}_pos"] = _to_np(p).reshape(-1)[:3]
                    payload[f"{side}_quat"] = _to_np(q).reshape(-1)[:4]
                except Exception as exc:  # noqa: BLE001
                    if _S.get("_ee_err", 0) < 2:
                        _S["_ee_err"] = _S.get("_ee_err", 0) + 1
                        _log(f"WARN 末端 {side} 取值失败: {exc!r}")
            # 物体位姿（world）：obj/<label>_pos / obj/<label>_quat
            for label, view in _S.get("obj_views", {}).items():
                try:
                    p, q = view.get_world_poses()
                    payload[f"obj/{label}_pos"] = _to_np(p).reshape(-1)[:3]
                    payload[f"obj/{label}_quat"] = _to_np(q).reshape(-1)[:4]
                except Exception as exc:  # noqa: BLE001
                    if _S.get("_obj_err", 0) < 2:
                        _S["_obj_err"] = _S.get("_obj_err", 0) + 1
                        _log(f"WARN 物体 {label} 取值失败: {exc!r}")

            # **不能用 savez_compressed**：实测 640x480 时压缩耗时 52ms，
            # 比渲染本身（19ms）还久，会把物理步进拖到 45ms/步。
            # savez 无压缩只要 1.3ms（快 40 倍），代价是文件大一点。
            np.savez(_S["out"] / "frames" / f"frame_{idx:06d}.npz", **payload)
            _S["saved"] += 1
            if idx % 20 == 0:
                _log(f"frame {idx} step {_S['step']} "
                     f"({_S['saved']} 帧已存, {_S['skipped']} 步跳过)")
        except Exception as exc:  # noqa: BLE001
            # 回调里绝不能抛异常，否则会打断物理步进 / 控制循环。
            # 但也不能静默 —— 首次失败必须把完整 traceback 打到 stdout，
            # 否则表现就是"物理步数在涨、帧数一直是 0"，极难定位。
            _S["errors"] = _S.get("errors", 0) + 1
            if _S["errors"] == 1:
                import traceback

                _log("!" * 60)
                _log(f"采帧失败（首次）: {exc!r}")
                traceback.print_exc()
                _log("!" * 60)
            carb.log_warn(f"[capture] 采帧失败: {exc}")

    sub = omni.physx.get_physx_interface().subscribe_physics_step_events(_on_physics_step)
    _S["sub"] = sub

    _log("=" * 60)
    _log(f"已挂载 —— 输出 {out}")
    _log(f"相机: {[c['name'] for c in _S['cameras']]}  每 {_S['every']} 个物理步采 1 帧")
    _log("现在去终端跑 motion-generation；结束后回来执行 stop_capture()")
    _log("=" * 60)
    return {"out": str(out), "cameras": [c["name"] for c in _S["cameras"]], "every": _S["every"]}


def stop_capture() -> dict:
    """停止采集并写 session.json。"""
    if not _S["armed"]:
        _log("未挂载")
        return {}

    try:
        if _S["sub"] is not None:
            _S["sub"].unsubscribe()
    except Exception as exc:  # noqa: BLE001
        _log(f"WARN 退订失败: {exc}")

    for cam in _S["cameras"]:
        try:
            cam["rp"].destroy()
        except Exception:  # noqa: BLE001
            pass

    out = _S["out"]
    info = {
        "frames_saved": _S["saved"],
        "physics_steps": _S["step"],
        "steps_skipped": _S["skipped"],
        "capture_errors": _S.get("errors", 0),
        "every": _S["every"],
        "duration_sec": (_S["t0"] is not None and __import__("time").monotonic() - _S["t0"]) or 0,
        "cameras": [
            {"name": c["name"], "prim": c["prim"], "size": c["size"]}
            for c in _S["cameras"]
        ],
        # 采了什么 —— 排查「字段没写进帧」时第一眼看这里
        "what": _S.get("what", []),
        "ee_captured": sorted(_S.get("ee_views", {}).keys()),
        "objects_captured": sorted(_S.get("obj_views", {}).keys()),
        "code_stamp": _S.get("code_stamp", "unknown"),
    }
    try:
        (out / "session.json").write_text(
            json.dumps(info, indent=2, ensure_ascii=False), encoding="utf-8"
        )
    except Exception as exc:  # noqa: BLE001
        _log(f"WARN 写 session.json 失败: {exc}")

    _log(f"停止 —— 共 {info['frames_saved']} 帧 / {info['physics_steps']} 物理步 "
         f"({info['duration_sec']:.1f}s)")
    _log(f"产物: {out}")
    if info["frames_saved"] == 0:
        _log("WARN 一帧都没存 —— 检查是否真的在跑仿真（timeline 是否 PLAY）")

    _S.update(armed=False, sub=None, cameras=[], out=None, records=[])
    return info


# Script Editor 里直接执行本文件时给一句提示，避免误以为自动开始了
if __name__ == "__main__":
    _log("已加载。调用 arm_capture(out_dir=..., every=10) 开始采集，"
         "stop_capture() 结束。")
