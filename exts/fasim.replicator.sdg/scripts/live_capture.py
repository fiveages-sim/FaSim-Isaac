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
    """按名字挑相机。

    **注意**：这里 ``camera`` 为空或 ``"all"`` 都表示"全部"（本函数的约定）。
    但 ``arm_capture`` 里空表示"不采" —— 由那里先判定，只在确定要采时才调
    本函数。不要直接拿用户输入调本函数。
    """
    specs = list(getattr(task, "cameras", None) or [])
    if not specs:
        return []
    if not camera or str(camera).strip().lower() == "all":
        return specs
    want = {c.strip() for c in str(camera).split(",") if c.strip()}
    picked = [s for s in specs if s.name in want]
    if not picked:
        _log(f"WARN 找不到相机 {want}，可选: {[s.name for s in specs]}")
    return picked


def _to_np(x: Any) -> np.ndarray:
    """warp array / torch tensor / numpy → numpy 数组。"""
    if hasattr(x, "numpy"):
        return np.asarray(x.numpy(), dtype=np.float64)
    return np.asarray(x, dtype=np.float64)


def _read_xform_world(prim_path: str) -> tuple[np.ndarray, np.ndarray] | None:
    """读一个 Xform prim 的**动态**世界位姿 (pos[3], quat[4])。

    **本项目里这个函数读不到动态值** —— 保留它只作对照诊断用。

    tcp 这类纯 Xform 帧不是刚体，默认 ``XformPrim.get_world_poses()`` 走
    ``usd`` 后端，读到的是 **authoring stage 的静态值** —— 物理跑起来后
    完全不变（实测 765 帧偏离 0.0mm）。

    切 ``fabric``/``usdrt`` 后端也救不了：newton kit 里
    ``updateToUsd=false`` / ``fabricUpdateTransformations=false``，
    PhysX 根本不把变换写回 USD/Fabric。

    真正的动态位姿见 ``_read_link_world``（读父刚体 + 固定偏移）。
    """
    from isaacsim.core.experimental.prims import XformPrim
    from isaacsim.core.experimental.utils import backend as backend_utils

    for backend in ("fabric", "usdrt", None):
        try:
            if backend is None:
                p, q = XformPrim(prim_path).get_world_poses()
            else:
                with backend_utils.use_backend(backend, raise_on_unsupported=True):
                    p, q = XformPrim(prim_path).get_world_poses()
            pos = _to_np(p).reshape(-1)[:3]
            quat = _to_np(q).reshape(-1)[:4]
            if np.all(np.isfinite(pos)):
                return pos, quat
        except Exception:  # noqa: BLE001
            continue
    return None


def _read_link_world(
    view: Any, offset: np.ndarray
) -> tuple[np.ndarray, np.ndarray] | None:
    """读刚体 link 的**动态**世界位姿，再叠固定偏移 → tcp 位姿。

    这是本项目末端位姿的**唯一可靠来源**。

    为什么可行
    ----------
    ``wrist_yaw_link`` 是 articulation 的刚体 link，``RigidPrim`` 走 tensor
    后端**直读 PhysX**，不经过 USD/Fabric 同步管道 —— 所以 newton kit 里
    关掉的那些 ``updateToUsd`` / ``fabricUpdateTransformations`` 开关对它
    没影响。（同一机制下物体位姿一直是准确的，可作对照。）

    tcp 相对 link 是 fixed joint 的常量变换，取自 URDF：沿 +X 平移
    ``0.0415`` m，无旋转。所以只需在 link 局部位姿上叠一个平移。

    Args:
        view: 已创建的 ``RigidPrim``。
        offset: link 系下的 tcp 偏移 (3,)。

    Returns:
        ``(pos[3], quat_wxyz[4])``；失败返回 ``None``。
    """
    try:
        p, q = view.get_world_poses()
        pos = _to_np(p).reshape(-1)[:3]
        quat = _to_np(q).reshape(-1)[:4]
        # pos 和 quat 都要校验 —— 只查 pos 会漏掉 NaN 姿态，
        # 后面拿坏四元数转偏移会算出全 NaN 的 tcp 位姿。
        if not (np.all(np.isfinite(pos)) and np.all(np.isfinite(quat))):
            return None
        # 局部偏移转到世界：p_tcp = p_link + R(q_link) @ offset
        off = np.asarray(offset, dtype=np.float64).reshape(3)
        if np.any(off):
            pos = pos + _quat_rotate(quat, off)
            if not np.all(np.isfinite(pos)):
                return None
        return pos, quat
    except Exception:  # noqa: BLE001
        return None


def _quat_rotate(q: np.ndarray, v: np.ndarray) -> np.ndarray:
    """用 wxyz 四元数旋转向量（右手系，与 Isaac/UsdGeom 一致）。"""
    w, x, y, z = (float(t) for t in np.asarray(q).reshape(-1)[:4])
    vx, vy, vz = (float(t) for t in np.asarray(v).reshape(-1)[:3])
    # t = 2 * (q_vec × v)
    tx = 2.0 * (y * vz - z * vy)
    ty = 2.0 * (z * vx - x * vz)
    tz = 2.0 * (x * vy - y * vx)
    return np.array([
        vx + w * tx + (y * tz - z * ty),
        vy + w * ty + (z * tx - x * tz),
        vz + w * tz + (x * ty - y * tx),
    ])


def _extract_camera_params(data: Any, width: int, height: int) -> dict[str, Any] | None:
    """从 Replicator ``camera_params`` 抽内参与相机世界位姿。

    实测字段（Isaac Sim 6.2，probe_camera_params.py 验证）::

        cameraFocalLength   = 18.1476      （**物理**焦距，mm/场景单位）
        cameraAperture      = [24.95, 13.936]   （视口尺寸，不是像素）
        cameraProjection    = 16 floats    （行主序 4x4 投影矩阵）
        cameraViewTransform = 16 floats    （**世界→相机** view matrix）
        renderProductResolution = [640, 480]

    **内参用 cameraProjection 推，不要用 focal/aperture**：本机 aperture
    是 16:9 而渲染分辨率 640x480 是 4:3，直接按 aperture 比例算会得到
    fx=465.5 / fy=625.1（错误）。投影矩阵里 ::

        cameraProjection[0] = 2*fx/W → fx = 0.5 * P[0] * W
        cameraProjection[5] = 2*fy/H → fy = 0.5 * P[5] * H

    实测两者都等于 465.51，才是正确的。

    **cameraViewTransform 是 view matrix（世界→相机），不是相机→世界**。
    实测验证：``R_view @ t_world`` ≈ ``-t_view``。所以相机世界位姿要求逆::

        R_cam_from_world = R_view.T
        t_cam = -R_cam_from_world @ t_view

    该矩阵是**行主序** 4x4 展平的（实测平移在 [3],[7],[11]）。

    Args:
        data: ``annotator.get_data()`` 的返回（dict）。
        width/height: 渲染分辨率。

    Returns:
        ``{"K": (3,3), "fx","fy","cx","cy", "pos": (3,), "quat": (4,) wxyz}``；
        解析失败返回 ``None``（不抛异常 —— 相机参数缺失不该让采集挂掉）。
    """
    if not isinstance(data, dict):
        return None
    try:
        proj = np.asarray(data.get("cameraProjection"), dtype=np.float64).reshape(-1)
        view = np.asarray(data.get("cameraViewTransform"), dtype=np.float64).reshape(-1)
        if proj.size < 16 or view.size < 16:
            return None
        # 行主序 4x4
        P = proj.reshape(4, 4)
        V = view.reshape(4, 4)

        # --- 内参：用投影矩阵反推，保证 fx/fy 与真实渲染一致 ---
        fx = 0.5 * abs(float(P[0, 0])) * width
        fy = 0.5 * abs(float(P[1, 1])) * height
        # 主点（正常应为 W/2, H/2；带上非对称 aperture 偏移时也能覆盖）
        cx = (0.5 * (1.0 + float(P[0, 2]))) * width
        cy = (0.5 * (1.0 + float(P[1, 2]))) * height
        if not (fx > 0 and fy > 0):
            return None
        K = np.array([[fx, 0.0, cx], [0.0, fy, cy], [0.0, 0.0, 1.0]])

        # --- 位姿：view matrix 求逆 ---
        R_vw = V[:3, :3]
        t_v = V[:3, 3]
        R_cw = R_vw.T
        # 视图矩阵的正交性可能因数值误差略微退化，做一次正交化
        U, _S, Vt = np.linalg.svd(R_cw)
        R_cw = U @ Vt
        if np.linalg.det(R_cw) < 0:
            U[:, -1] *= -1
            R_cw = U @ Vt
        pos = -R_cw @ t_v
        quat = _rot_to_wxyz(R_cw)
        if not (np.all(np.isfinite(pos)) and np.all(np.isfinite(quat))):
            return None
        return {
            "K": K,
            "fx": float(fx), "fy": float(fy),
            "cx": float(cx), "cy": float(cy),
            "pos": pos, "quat": quat,
            "model": str(data.get("cameraModel") or ""),
        }
    except Exception:  # noqa: BLE001
        return None


def _rot_to_wxyz(R: np.ndarray) -> np.ndarray:
    """旋转矩阵 → 四元数 (w,x,y,z)。"""
    R = np.asarray(R, dtype=np.float64)
    tr = R[0, 0] + R[1, 1] + R[2, 2]
    if tr > 0:
        s = np.sqrt(tr + 1.0) * 2
        w = 0.25 * s
        x = (R[2, 1] - R[1, 2]) / s
        y = (R[0, 2] - R[2, 0]) / s
        z = (R[1, 0] - R[0, 1]) / s
    elif R[0, 0] > R[1, 1] and R[0, 0] > R[2, 2]:
        s = np.sqrt(1.0 + R[0, 0] - R[1, 1] - R[2, 2]) * 2
        w = (R[2, 1] - R[1, 2]) / s
        x = 0.25 * s
        y = (R[0, 1] + R[1, 0]) / s
        z = (R[0, 2] + R[2, 0]) / s
    elif R[1, 1] > R[2, 2]:
        s = np.sqrt(1.0 + R[1, 1] - R[0, 0] - R[2, 2]) * 2
        w = (R[0, 2] - R[2, 0]) / s
        x = (R[0, 1] + R[1, 0]) / s
        y = 0.25 * s
        z = (R[1, 2] + R[2, 1]) / s
    else:
        s = np.sqrt(1.0 + R[2, 2] - R[0, 0] - R[1, 1]) * 2
        w = (R[1, 0] - R[0, 1]) / s
        x = (R[0, 2] + R[2, 0]) / s
        y = (R[1, 2] + R[2, 1]) / s
        z = 0.25 * s
    q = np.array([w, x, y, z], dtype=np.float64)
    n = float(np.linalg.norm(q))
    return q / n if n > 0 else np.array([1.0, 0.0, 0.0, 0.0])


def _probe_xform_backend(prim_path: str) -> tuple[bool, str]:
    """自检：哪个后端能读到该 Xform 的动态世界位姿。

    Returns:
        ``(ok, 后端名)``。``ok=False`` 表示读不到（只能拿静态值）。
    """
    from isaacsim.core.experimental.prims import XformPrim
    from isaacsim.core.experimental.utils import backend as backend_utils

    for backend in ("fabric", "usdrt"):
        try:
            with backend_utils.use_backend(backend, raise_on_unsupported=True):
                p, _q = XformPrim(prim_path).get_world_poses()
            pos = _to_np(p).reshape(-1)[:3]
            if np.all(np.isfinite(pos)):
                return True, backend
        except Exception:  # noqa: BLE001
            continue
    return False, "usd(静态)"


def _pose_delta_mm(
    a: tuple[np.ndarray, np.ndarray] | None,
    b: tuple[np.ndarray, np.ndarray] | None,
) -> float:
    """两个位姿的位置差（mm）；任一无效返回 nan。"""
    if a is None or b is None:
        return float("nan")
    return float(np.linalg.norm(np.asarray(a[0]) - np.asarray(b[0])) * 1000.0)


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

    # 相机：``camera`` 的语义（务必与 CLI 保持一致，否则会静默不采）
    #   None / ""      → **不采相机**（只采状态），不建 render product
    #   "all"          → 采 task.yaml 里配的全部相机
    #   "head,left"    → 按名字过滤
    #
    # 注意历史坑：``_camera_specs`` 内部把 "" 当"全部"，而这里是""当"不采"，
    # 同一空串两种含义导致「选全部相机」被当成「不采」。现在由本处统一判定，
    # 只有非空才往下走，并把 "all" 映射成空串再交给 _camera_specs。
    cams: list[dict] = []
    cam_sel = str(camera if camera is not None else "").strip()
    if cam_sel:
        specs = _camera_specs(task, "" if cam_sel.lower() == "all" else cam_sel)
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
            # 相机参数（内参 + 位姿）—— 没有它深度图无法转点云，也无法做
            # 视觉-状态对齐校验。字段与布局见 _extract_camera_params。
            cp = None
            try:
                cp = rep.annotators.get("camera_params")
                cp.attach(rp)
            except Exception as exc:  # noqa: BLE001
                _log(f"WARN 相机 {spec.name} 的 camera_params 不可用: {exc}")
                cp = None
            cams.append({"name": spec.name, "rp": rp, "rgb": rgb, "depth": dep,
                         "cparams": cp,
                         "size": (w, h), "prim": str(spec.prim_path)})
            _log(f"相机 {spec.name}: {w}x{h} @ {spec.prim_path}"
                 + ("  (+内参)" if cp is not None else ""))
        if not cams:
            _log("ERROR 所有相机都创建失败")
            return None
    else:
        # 明确说清"为什么没采"—— cam02 就是选了全部相机却因空串语义丢成
        # 不采，事后才从帧里发现没图像。这里把判定依据打出来。
        _log(f"不采相机（camera={camera!r}；只采状态）")

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

    # 末端位姿（tcp，world 系）—— 两套来源，**同时采**以便一次定位问题。
    #
    # 1) ee_links: 读父刚体 + 固定偏移 → **动态真值**（RigidPrim 直读 PhysX）
    # 2) ee_prims: 直接读 tcp Xform → 已知是静态值，只作对照
    #
    # 为什么两套都采：本次调试的核心疑问就是「tcp 位姿到底采没采到」。把
    # Xform 静态值和刚体动态值同时写进帧里，事后一眼就能看出哪个是死的、
    # 差多少、差在哪个方向 —— 不用再来回重启试探。
    ee_links: dict[str, Any] = {}
    ee_offsets: dict[str, np.ndarray] = {}
    ee_prims: dict[str, str] = {}
    if record_ee:
        from isaacsim.core.experimental.prims import RigidPrim

        for side, spec in (getattr(robot, "ee_links", None) or {}).items():
            link = str(spec.get("link") or "")
            if not link:
                continue
            off = np.asarray(
                spec.get("offset") or (0.0, 0.0, 0.0), dtype=np.float64
            ).reshape(3)
            view = None
            try:
                view = RigidPrim(link)
            except Exception as exc:  # noqa: BLE001
                _log(f"  末端 {side}: RigidPrim 失败（{exc}），试 XformPrim 兜底")
                # articulation 的 link 有时不能当独立刚体包装；XformPrim 走
                # usdrt/fabric 也可能读不到（正是本项目的问题）。兜底只为
                # 不至于整个采集挂掉，正确性由后面的自检报告判定。
                try:
                    from isaacsim.core.experimental.prims import XformPrim

                    view = XformPrim(link)
                except Exception as exc2:  # noqa: BLE001
                    _log(f"WARN 末端 {side} 两种视图都失败（{link}）: {exc2}")
                    continue
            ee_links[side] = view
            ee_offsets[side] = off

        for side, prim in (getattr(robot, "ee_prims", None) or {}).items():
            ee_prims[side] = str(prim)

        if ee_links:
            _log(f"末端位姿: {list(ee_links)}（刚体+偏移，动态值）"
                 f" 偏移={ {s: v.tolist() for s, v in ee_offsets.items()} }")
            # 自检：确认刚体真的能读到（不依赖后续物理）
            try:
                s0 = next(iter(ee_links))
                got = _read_link_world(ee_links[s0], ee_offsets[s0])
                _log(f"  自检 {s0} link 可读: "
                     + (f"pos={np.round(got[0], 4).tolist()}" if got else "失败！"))
            except Exception as exc:  # noqa: BLE001
                _log(f"  WARN 自检失败: {exc}")
        else:
            _log("WARN 没有可用的末端刚体 —— 检查 robot.yaml 的 ee_links")
        if ee_prims:
            _log(f"末端对照: {list(ee_prims)}（Xform，静态值，仅用于对比）")

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
    if ee_links:
        what.append("ee")
    if obj_views:
        what.append("object")
    if not what:
        _log("ERROR 什么都没采（相机/关节/末端/物体全部不可用）")
        return None

    import time

    # 记录**真实**物理步长 —— pack 用它算帧间隔，假设 5ms 会导致速度偏差。
    # 实测这台机器物理步长 5.876ms（177Hz 而非 200Hz），若按 5ms 换算，
    # 回放会快 15%，手到得早、位置偏。
    physics_dt = 0.0
    try:
        from isaacsim.core.simulation_manager import SimulationManager

        physics_dt = float(SimulationManager.get_physics_dt())
        if physics_dt <= 0:
            physics_dt = 0.0
    except Exception as exc:  # noqa: BLE001
        _log(f"WARN 读物理步长失败（pack 将回退假设 5ms）: {exc}")

    _S.update(
        armed=True, cameras=cams, out=out, every=max(1, int(every)),
        step=0, saved=0, skipped=0, t0=time.monotonic(), records=[],
        size=cams[0]["size"] if cams else (0, 0),
        ee_links=ee_links, ee_offsets=ee_offsets, ee_prims=ee_prims,
        obj_views=obj_views, what=what,
        code_stamp=_code_stamp(),
        physics_dt=physics_dt,
        # 供 session.json 记录，下游 pack/fk 靠它定位 planning URDF
        robot_key=str(robot_key), task_key=str(task_key),
    )
    if physics_dt > 0:
        _log(f"物理步长 {physics_dt*1000:.3f} ms（{1/physics_dt:.1f} Hz）"
             f"→ 每帧 {physics_dt*max(1,int(every))*1000:.2f} ms")

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
                # 相机内参 + 世界位姿 —— 每帧都记：腕部相机是**随手臂动的**，
                # 只记一次不够。头相机虽固定，统一处理避免特例。
                cpv = cam.get("cparams")
                if cpv is not None:
                    try:
                        got = _extract_camera_params(
                            cpv.get_data(), int(cam["size"][0]), int(cam["size"][1])
                        )
                    except Exception:  # noqa: BLE001
                        got = None
                    if got is not None:
                        name = cam["name"]
                        payload[f"{name}_K"] = got["K"]
                        payload[f"{name}_pos"] = got["pos"]
                        payload[f"{name}_quat"] = got["quat"]
                        h = _S.setdefault("cam_health", {}).setdefault(
                            name, {"fx": got["fx"], "fy": got["fy"],
                                   "first_pos": [round(float(x), 4) for x in got["pos"]],
                                   "max_travel_mm": 0.0, "samples": 0}
                        )
                        h["max_travel_mm"] = max(
                            h["max_travel_mm"],
                            float(np.linalg.norm(
                                got["pos"] - np.asarray(h["first_pos"])) * 1000.0),
                        )
                        h["samples"] += 1
                    elif _S.get(f"_cp_err_{cam['name']}", 0) < 2:
                        _S[f"_cp_err_{cam['name']}"] = 1
                        _log(f"WARN 相机 {cam['name']} 参数解析失败（内参将缺失）")
            if art is not None:
                try:
                    q = art.get_dof_positions()
                    payload["joint_pos"] = _to_np(q).astype(np.float32).reshape(1, -1)
                    payload["joint_names"] = dof_names
                except Exception:  # noqa: BLE001
                    pass
                # **下发目标**关节角 —— 回放要用这个，不是实测值。
                #
                # 编排时 OCS2 把 q_cmd 写给 drive，物理跟上到 q_act = q_cmd − e
                # （手臂自重造成稳态误差 e）。若采集记 q_act、回放又把它当
                # 目标驱动一次，e 会被叠加两次，末端 = J·e，手臂前伸时主要
                # 指向下方 —— 正是"抓试管稳定偏下 2-3cm"的成因。
                try:
                    qt = art.get_dof_position_targets()
                    payload["joint_pos_cmd"] = _to_np(qt).astype(np.float32).reshape(1, -1)
                    _S["joint_cmd_ok"] = True
                except Exception as exc:  # noqa: BLE001
                    if _S.get("_qt_err", 0) < 2:
                        _S["_qt_err"] = _S.get("_qt_err", 0) + 1
                        _log(f"WARN 读 drive 目标失败（回放将回退用实测值，"
                             f"末端会多叠一次稳态误差）: {exc}")
            # 末端位姿（world）—— 主值走刚体+偏移（动态真值）
            #   <side>_pos / <side>_quat          ← 刚体读，**训练用这个**
            #   <side>_pos_xform / _quat_xform    ← tcp Xform 直读，静态对照值
            #
            # 同时写两套是刻意的：调试期一眼能看出哪个是死的、差多少、差在
            # 哪个方向。确认无问题后可以关掉 _xform 那套省体积。
            for side, view in _S.get("ee_links", {}).items():
                got = _read_link_world(view, _S["ee_offsets"][side])
                if got is None:
                    if _S.get("_ee_err", 0) < 2:
                        _S["_ee_err"] = _S.get("_ee_err", 0) + 1
                        _log(f"WARN 末端 {side} 刚体读取失败")
                    continue
                payload[f"{side}_pos"] = got[0]
                payload[f"{side}_quat"] = got[1]
                # 累计自检：首帧值 + 与首帧的最大偏离 —— 直接回答
                # 「这个值到底动没动」。死值会一直停在 0.0mm。
                h = _S.setdefault("ee_health", {}).setdefault(
                    side, {"first": None, "max_travel_mm": 0.0, "samples": 0}
                )
                p = got[0]
                if h["first"] is None:
                    h["first"] = [round(float(x), 5) for x in p]
                h["max_travel_mm"] = max(
                    h["max_travel_mm"],
                    float(np.linalg.norm(p - np.asarray(h["first"])) * 1000.0),
                )
                h["samples"] += 1
            for side, prim_path in _S.get("ee_prims", {}).items():
                got = _read_xform_world(prim_path)
                if got is not None:
                    payload[f"{side}_pos_xform"] = got[0]
                    payload[f"{side}_quat_xform"] = got[1]
                    # 对照自检：Xform 静态值同样统计，两相对照即可定性
                    hx = _S.setdefault("ee_health", {}).setdefault(
                        f"{side}_xform",
                        {"first": None, "max_travel_mm": 0.0, "samples": 0},
                    )
                    px = got[0]
                    if hx["first"] is None:
                        hx["first"] = [round(float(x), 5) for x in px]
                    hx["max_travel_mm"] = max(
                        hx["max_travel_mm"],
                        float(np.linalg.norm(px - np.asarray(hx["first"])) * 1000.0),
                    )
                    hx["samples"] += 1
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
        "out": str(out),
        # 机器人/任务 key —— 下游（pack/fk）靠它找 configs/<robot>/robot.yaml，
        # 决定反算末端指令位姿用哪份 planning URDF。缺了就只能让用户手填。
        "robot_key": _S.get("robot_key", ""),
        "task_key": _S.get("task_key", ""),
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
        "ee_captured": sorted(_S.get("ee_links", {}).keys()),
        "ee_offsets": {
            s: [round(float(x), 6) for x in v]
            for s, v in (_S.get("ee_offsets") or {}).items()
        },
        "ee_xform_control": sorted(_S.get("ee_prims", {}).keys()),
        "objects_captured": sorted(_S.get("obj_views", {}).keys()),
        "code_stamp": _S.get("code_stamp", "unknown"),
        "physics_dt": _S.get("physics_dt", 0.0),
        # 采集期逐帧自检（不落盘，只在内存里累计）—— 收工时一次看清
        "ee_health": _S.get("ee_health", {}),
        "camera_health": _S.get("cam_health", {}),
        "joint_cmd_recorded": bool(_S.get("joint_cmd_ok", False)),
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

    _log_ee_health(_S.get("ee_health") or {})
    _log_camera_health(_S.get("cam_health") or {})
    if not info["joint_cmd_recorded"]:
        _log("WARN 没采到 joint_pos_cmd（下发目标）—— 回放只能回退实测值")

    _S.update(armed=False, sub=None, cameras=[], out=None, records=[])
    return info


def _log_camera_health(health: dict) -> None:
    """相机自检结论。判据同样看「值动了没有」+ 内参是否合理。"""
    if not health:
        return
    _log("-" * 60)
    _log("相机自检")
    for name, h in sorted(health.items()):
        fx, fy = h.get("fx", 0.0), h.get("fy", 0.0)
        travel = h.get("max_travel_mm", 0.0)
        # 腕部相机会跟着手走（百毫米级）；头相机基本固定（几十毫米内，
        # 因为躯干会轻微晃动）。所以这里不判死值，只报告位移。
        k_ok = fx > 0 and fy > 0 and abs(fx - fy) / max(fx, fy) < 0.02
        _log(f"  {name:12s} fx={fx:7.2f} fy={fy:7.2f}"
             + ("  内参一致 ✅" if k_ok else "  ⚠ fx/fy 差异大")
             + f"  相机位移={travel:7.1f}mm  ({h.get('samples', 0)} 帧)")
    _log("  注: 头相机位移小是正常的（躯干微动）；腕部相机应随手臂大幅移动")
    _log("-" * 60)


def _log_ee_health(health: dict) -> None:
    """把末端采集自检打成结论，不用人去读 json。

    判据很直接：**值动了没有**。死值（读到静态 authoring 值）会一直停在
    0.0mm，而动值应该跟末端行程同量级。
    """
    if not health:
        return
    _log("-" * 60)
    _log("末端采集自检（max_travel = 与首帧的最大偏离）")
    for name, h in sorted(health.items()):
        t = h.get("max_travel_mm", 0.0)
        tag = "动态 ✅" if t >= 10.0 else "静态 ❌"
        _log(f"  {name:12s} {tag}  travel={t:7.1f}mm  首帧={h.get('first')}"
             f"  ({h.get('samples', 0)} 次)")
    # 结论：主值与对照值对比
    prim = {k: v for k, v in health.items() if not k.endswith("_xform")}
    xf = {k: v for k, v in health.items() if k.endswith("_xform")}
    if prim and xf:
        pmax = max(v.get("max_travel_mm", 0.0) for v in prim.values())
        xmax = max(v.get("max_travel_mm", 0.0) for v in xf.values())
        _log(f"  刚体读取 max={pmax:.1f}mm   Xform 对照 max={xmax:.1f}mm")
        if pmax >= 10.0 and xmax < 1.0:
            _log("  → 结论：刚体读取是动态的，Xform 果然是死值。"
                 "末端现在采对了（用 <side>_pos）。")
        elif pmax < 10.0:
            _log("  → 结论：刚体读取也是死的！检查 ee_links 指向的 link "
                 "是否真是 articulation 的刚体。")
    _log("-" * 60)


# Script Editor 里直接执行本文件时给一句提示，避免误以为自动开始了
if __name__ == "__main__":
    _log("已加载。调用 arm_capture(out_dir=..., every=10) 开始采集，"
         "stop_capture() 结束。")
