"""在**正在运行编排**的 Isaac Sim 会话里旁路采集 Replicator 数据。

用途
====
路线：编排当主干（闭环、高成功率），Replicator 当工具（出相机数据）。
不额外起 ROS 通信、不接管控制——只挂在同一个 Isaac 进程里"看"。

为什么能不影响编排
==================
ros2_control（``ROS=fiveages`` 变体）跑在 **Isaac 进程内**，和渲染循环
共享同一个 ``app.update()`` 时钟。本模块：

- **不自己开循环**，挂 ``physics step`` 事件回调；
- 回调里只做 ``annotator.get_data()``（读已有缓冲），不阻塞；
- 渲染用 ``orchestrator.step()``，但按 ``--every`` 降采样（默认每 10 个
  物理步才渲一次），把开销压到可控。

关键：``get_data()`` 拿到的永远是**最近一次渲染**的缓冲。所以必须保证
每次都真正渲过（``orchestrator.step``），否则会拿到重复帧。脚本会记录
时间戳，跑完可以检查是否有重复。

用法（Isaac Sim GUI 已启动、场景已加载、准备跑编排时）
====================================================
在 ``Window > Script Editor`` 里粘贴执行::

    import sys
    sys.path.insert(0, "/home/fiveages/FaSim-Isaac/exts/fasim.replicator.sdg")
    exec(open("/home/fiveages/FaSim-Isaac/exts/fasim.replicator.sdg/scripts/live_capture.py").read())

    arm_capture(out_dir="~/fasim_live", every=10, camera="head")
    # -> 立刻返回。现在去终端跑 motion-generation。

    # 编排跑完后：
    stop_capture()

产物::

    <out_dir>/frames/frame_000123.npz    rgb / depth / 内参 / 时间戳
    <out_dir>/session.json               相机参数、帧数、丢帧统计

**不要在 Script Editor 里写 while/sleep 循环** —— 会饿死 Kit 主循环，
物理停住、OCS2 卡死。
"""

from __future__ import annotations

import json
import sys
from pathlib import Path
from typing import Any

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


def arm_capture(
    out_dir: str = "~/fasim_live",
    every: int = 10,
    camera: str = "head",
    robot_key: str = "unitree_g1d",
    task_key: str = "sinopec_test_tube",
    width: int | None = None,
    height: int | None = None,
    record_joints: bool = True,
    depth: bool = True,
) -> dict | None:
    """挂载采集器并立即返回（非阻塞）。

    Args:
        out_dir: 输出根目录。
        every: 每多少个**物理步**采一帧。物理 200Hz 时 every=10 ≈ 20fps。
            渲染一次约 20~25ms；若发现控制明显变慢，把它调大（如 20 或 40）。
        camera: 相机名，逗号分隔多个；空字符串=全部。
        robot_key / task_key: 选 ``configs/<robot>/`` 与 ``tasks/<task>.yaml``。
        width/height: 覆盖分辨率；None 用配置值。**降到 320x240 可显著降开销**。
        record_joints: 是否顺带记录关节角（作为 action 标签）。
        depth: 是否采深度。关掉能省一些渲染开销。

    Returns:
        ``{"out":..., "cameras":[...], "every":...}``；已挂载则返回 None。
    """
    if _S["armed"]:
        _log("已挂载，先调用 stop_capture()")
        return None

    SDG_EXT = Path("/home/fiveages/FaSim-Isaac/exts/fasim.replicator.sdg")
    if str(SDG_EXT) not in sys.path:
        sys.path.insert(0, str(SDG_EXT))

    import carb
    import omni.physx
    import omni.replicator.core as rep
    import omni.usd

    from fasim.replicator.sdg.config_io import load_robot_config, load_task_config

    robot = load_robot_config(robot_key)
    task = load_task_config(robot_key, task_key)

    out = Path(out_dir).expanduser()
    (out / "frames").mkdir(parents=True, exist_ok=True)

    specs = _camera_specs(task, camera)
    if not specs:
        _log("ERROR 没有可用相机")
        return None

    cams: list[dict] = []
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

    # 关节角（作为 action 标签）。拿不到就跳过，不影响图像采集。
    art = None
    dof_names: list[str] = []
    if record_joints:
        try:
            from isaacsim.core.experimental.prims import Articulation

            art = Articulation(robot.articulation_prim)
            dof_names = list(art.dof_names)
            _log(f"关节标签: {len(dof_names)} 个 DOF")
        except Exception as exc:  # noqa: BLE001
            _log(f"WARN 关节读取不可用，只采图像: {exc}")
            art = None

    import time

    _S.update(
        armed=True, cameras=cams, out=out, every=max(1, int(every)),
        step=0, saved=0, skipped=0, t0=time.monotonic(), records=[],
        size=cams[0]["size"],
    )

    def _on_physics_step(dt: float) -> None:
        """物理步进回调：按 every 降采样，渲染并落盘。必须快速返回。"""
        _S["step"] += 1
        if _S["step"] % _S["every"] != 0:
            _S["skipped"] += 1
            return
        try:
            # 真正渲一帧（否则 annotator 会返回上一次的缓冲，出现重复帧）
            rep.orchestrator.step(delta_time=0.0, pause_timeline=False)
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
                    payload["joint_pos"] = q.numpy() if hasattr(q, "numpy") else q
                    payload["joint_names"] = dof_names
                except Exception:  # noqa: BLE001
                    pass
            import numpy as np

            # **不能用 savez_compressed**：实测 640x480 时压缩耗时 52ms，
            # 比渲染本身（19ms）还久，会把物理步进拖到 45ms/步。
            # savez 无压缩只要 1.3ms（快 40 倍），代价是文件大一点。
            np.savez(_S["out"] / "frames" / f"frame_{idx:06d}.npz", **payload)
            _S["saved"] += 1
            if idx % 20 == 0:
                _log(f"frame {idx} step {_S['step']} "
                     f"({_S['saved']} 帧已存, {_S['skipped']} 步跳过)")
        except Exception as exc:  # noqa: BLE001
            # 回调里绝不能抛异常，否则会打断物理步进 / 控制循环
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
        "every": _S["every"],
        "duration_sec": (_S["t0"] is not None and __import__("time").monotonic() - _S["t0"]) or 0,
        "cameras": [
            {"name": c["name"], "prim": c["prim"], "size": c["size"]}
            for c in _S["cameras"]
        ],
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
