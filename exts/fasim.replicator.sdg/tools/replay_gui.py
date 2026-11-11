"""交互式回放调试：带 GUI 视窗，可拖动/暂停/单步，直接看回放效果。

用途
====
``verify_replay.py`` 是无头抽样出图的，看不到连续动作。这个脚本开窗口，
按真实帧率回放 episode 的关节角，夹爪走 drive（与运控同一协议），
物体由物理自然响应 —— 与你手动跑编排时的场景完全一致（保留 ROS 变体）。

用法::

    /home/fiveages/isaacsim/python.sh tools/replay_gui.py --episode ~/fa_data/pick_10.npz

    # 只看某一段（例如抓取前后）
    ... tools/replay_gui.py --episode ~/fa_data/pick_10.npz --start 800 --end 1400

    # 慢速播放，方便观察接触
    ... tools/replay_gui.py --episode ~/fa_data/pick_10.npz --speed 0.3

操作
====
脚本会自动把视窗相机切到俯视第三人称机位。回放过程中：
  - 空格：暂停 / 继续
  - 关闭窗口即退出

产物::

    <out>/replay_gui_report.json   每帧的关节误差、试管位姿偏差
"""

from __future__ import annotations

import argparse
import json
import sys
import time
from pathlib import Path

import numpy as np

SDG_EXT = Path(__file__).resolve().parents[1]
if str(SDG_EXT) not in sys.path:
    sys.path.insert(0, str(SDG_EXT))


def _log(msg: str) -> None:
    print(f"[gui] {msg}", flush=True)


def main() -> int:
    ap = argparse.ArgumentParser(description="交互式回放（带视窗）")
    ap.add_argument("--episode", required=True)
    ap.add_argument("--robot", default="", help="机器人 key（留空=自动选唯一配置）")
    ap.add_argument("--task", default="", help="任务 key（留空=自动选唯一配置）")
    ap.add_argument("--out", default="/tmp/replay_gui")
    ap.add_argument("--start", type=int, default=0)
    ap.add_argument("--end", type=int, default=0, help="0 = 到末尾")
    ap.add_argument("--speed", type=float, default=1.0, help="1.0 = 实时")
    ap.add_argument(
        "--gripper-mode",
        choices=("drive", "teleport", "none"),
        default="drive",
        help="drive=走 set_dof_position_targets（推荐，与运控一致）；"
        "teleport=直接设位置；none=不动夹爪",
    )
    ap.add_argument(
        "--keep-ros",
        action="store_true",
        default=True,
        help="保留 ROS 变体（默认 true，与录制场景一致）",
    )
    ap.add_argument(
        "--no-ros",
        dest="keep_ros",
        action="store_false",
        help="去掉 ROS 变体（纯渲染，不跑 ros2_control）",
    )
    ap.add_argument(
        "--camera",
        default="",
        help="切到哪个相机的视角（默认 head）；留空=不改视窗",
    )
    ap.add_argument(
        "--no-lock-base",
        dest="lock_base",
        action="store_false",
        default=True,
        help="不锁底盘（默认锁，与录制时的 BASE_LOCK 一致）。用来对比漂移影响",
    )
    ap.add_argument(
        "--arm-mode",
        choices=("drive", "teleport"),
        default="drive",
        help="手臂驱动方式。drive=set_dof_position_targets+gain（与运控一致，"
        "接触稳定，能夹起物体）；teleport=set_dof_positions（瞬移，接触会失效）",
    )
    ap.add_argument("--arm-stiffness", type=float, default=100000.0)
    ap.add_argument("--arm-damping", type=float, default=10000.0)
    ap.add_argument(
        "--grip-stiffness",
        type=float,
        default=50000.0,
        help="夹爪 drive 刚度。资产默认 10000 太软，竖直提起会滑落",
    )
    ap.add_argument("--grip-damping", type=float, default=2000.0)
    ap.add_argument(
        "--grip-maxforce",
        type=float,
        default=25.0,
        help="夹爪最大夹持力(N)。实测临界点在 20~25N：20N 顶不住会掉，"
        "25N 稳定。Dex1 额定 effort 是 20N，取 25N 尽量贴近现实",
    )
    ap.add_argument(
        "--close-target",
        type=float,
        default=None,
        help="夹爪闭合目标（覆盖默认的关节下限）。默认用下限，与运控一致",
    )
    args = ap.parse_args()

    out = Path(args.out).expanduser()
    out.mkdir(parents=True, exist_ok=True)

    # ---- 先读数据（不需要 Isaac）----
    from fasim.replicator.sdg.config_io import (
        CONFIG_ROOT,
        load_robot_config,
        load_task_config,
    )
    from fasim.replicator.sdg.episode import Episode, build_dof_mapping

    root = CONFIG_ROOT
    robot_cfg = load_robot_config(args.robot, root)
    task_cfg = load_task_config(args.robot, args.task, root)
    ep = Episode.load(args.episode)
    n = len(ep)
    n_start = max(0, args.start)
    n_end = n if args.end <= 0 else min(n, args.end)
    _log(f"episode: {n} 帧，回放 [{n_start}, {n_end})")

    # ---- 开视窗 ----
    from isaacsim import SimulationApp

    app = SimulationApp(launch_config={"headless": False})
    _log("视窗已启动")

    import omni.timeline
    import omni.usd

    from fasim.replicator.sdg.recorder import _apply_variants, _open_stage
    from fasim.replicator.sdg.renderer import _lock_base

    _open_stage(Path(task_cfg.scene_usda))
    variants = dict(getattr(task_cfg, "variant_selection", {}) or {})
    if not args.keep_ros:
        variants.pop("ROS", None)
    _log(f"variants: {variants}")
    if variants:
        _apply_variants(robot_cfg.articulation_prim, variants)

    # 锁底盘：复现录制时的 BASE_LOCK（必须在 Articulation 之前）
    if args.lock_base:
        _lock_base(robot_cfg)
    else:
        _log("未锁底盘（--no-lock-base），会观察手臂反作用力引起的漂移")

    # 夹爪 drive：力量不够竖直提不起来（资产默认 maxForce=20N 会滑落）
    try:
        from pxr import Usd as _Usd

        import omni.usd as _ou

        stage = _ou.get_context().get_stage()
        n_set = 0
        for prim in _Usd.PrimRange(stage.GetPrimAtPath(robot_cfg.articulation_prim)):
            if not prim.GetName().lower().endswith("gripper_joint"):
                continue
            if not prim.GetAttribute("drive:linear:physics:type").IsValid():
                continue
            for attr, val in (
                ("drive:linear:physics:stiffness", args.grip_stiffness),
                ("drive:linear:physics:damping", args.grip_damping),
                ("drive:linear:physics:maxForce", args.grip_maxforce),
            ):
                a = prim.GetAttribute(attr)
                if a and a.IsValid():
                    a.Set(float(val))
            n_set += 1
        _log(
            f"夹爪 drive: K={args.grip_stiffness:g} D={args.grip_damping:g} "
            f"maxForce={args.grip_maxforce:g}N（{n_set} 个关节）"
        )
    except Exception as exc:  # noqa: BLE001
        _log(f"WARN 设置夹爪 drive 失败: {exc}")

    for _ in range(10):
        app.update()

    from isaacsim.core.experimental.prims import Articulation, RigidPrim

    art = Articulation(robot_cfg.articulation_prim)
    dof_names = list(art.dof_names)
    _log(f"articulation DOF={len(dof_names)}")

    target_idx, source_idx, missing = build_dof_mapping(dof_names, ep.joint_names)
    if missing:
        _log(f"WARN 找不到的关节: {missing}")
    # 剔除整列 NaN（mimic 关节由 mimic API 跟随）
    if len(source_idx):
        ok = np.isfinite(ep.joint_pos[:, source_idx]).any(axis=0)
        dead = [ep.joint_names[c] for c, k in zip(source_idx, ok) if not k]
        if dead:
            _log(f"跳过整列 NaN 的关节: {dead}")
            target_idx = target_idx[ok]
            source_idx = source_idx[ok]

    # 夹爪单独处理
    from fasim.replicator.sdg.episode import GRIPPER_NAME_TOKENS

    name_of = lambda k: str(ep.joint_names[source_idx[k]])  # noqa: E731
    grip_k = [
        k for k in range(len(source_idx))
        if any(t in name_of(k).lower() for t in GRIPPER_NAME_TOKENS)
    ]
    arm_k = [k for k in range(len(source_idx)) if k not in grip_k]
    _log(f"手臂关节 {len(arm_k)}，夹爪 {[name_of(k) for k in grip_k]}")

    grip_targets: dict[int, tuple[float, float]] = {}
    # 关节限位：运控的 closed_position_ 就是它（见 AdaptiveGripperController
    # ::parse_initial_value），闭合时手指被物体挡住 ⇒ target-actual≠0 ⇒ 持续出力。
    lim_lo = lim_hi = None
    try:
        lims = art.get_dof_limits()
        lim_lo = np.asarray(lims[0], dtype=np.float64).reshape(-1)
        lim_hi = np.asarray(lims[1], dtype=np.float64).reshape(-1)
    except Exception as exc:  # noqa: BLE001
        _log(f"WARN 读关节限位失败: {exc}")

    for k in grip_k:
        vals = np.asarray(ep.joint_pos[:, source_idx[k]], dtype=np.float64)
        vals = vals[np.isfinite(vals)]
        if vals.size == 0:
            continue
        dof = int(target_idx[k])
        o = float(np.percentile(vals, 98))
        # 默认闭合目标 = 关节下限（与运控一致），有物体挡着就用不上全部行程
        c = float(lim_lo[dof]) if lim_lo is not None else float(np.percentile(vals, 2))
        if args.close_target is not None:
            c = float(args.close_target)
        grip_targets[dof] = (o, c)
        rec_c = float(np.percentile(vals, 2))
        _log(
            f"夹爪 {name_of(k)}: 开 {o:+.4f} / 合(目标) {c:+.4f} ；"
            f"录制里闭合时实际停在 {rec_c:+.4f}（被物体挡住的位置）"
        )

    # 物体
    obj_views: dict[str, RigidPrim] = {}
    prim_map = dict(getattr(task_cfg, "tracked_prims", {}) or {})
    for label in (ep.objects or {}):
        p = prim_map.get(label)
        if not p:
            continue
        try:
            obj_views[label] = RigidPrim(p)
        except Exception as exc:  # noqa: BLE001
            _log(f"WARN 物体 {label} 无法创建: {exc}")

    # 底盘位姿监控：量化漂移（偏航角 + 位移），这是判断「手臂是否落空」的
    # 第一指标。录制时 BASE_LOCK 下它应始终为 0。
    base_view = None
    base_p0 = base_q0 = None
    base_path = str(getattr(robot_cfg, "base_prim", "") or "")
    if base_path:
        try:
            from isaacsim.core.experimental.prims import XformPrim

            base_view = XformPrim(base_path)
        except Exception as exc:  # noqa: BLE001
            _log(f"WARN 无法监控 base: {exc}")

    def _yaw_deg(q) -> float:
        q = np.asarray(q).reshape(-1)
        w, x, y, z = float(q[0]), float(q[1]), float(q[2]), float(q[3])
        return float(np.degrees(np.arctan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))))

    def _base_drift() -> tuple[float, float]:
        nonlocal base_p0, base_q0
        if base_view is None:
            return 0.0, 0.0
        try:
            p, q = base_view.get_world_poses()
            p = np.asarray(p).reshape(-1)
            if base_p0 is None:
                base_p0, base_q0 = p.copy(), np.asarray(q).reshape(-1).copy()
                return 0.0, 0.0
            dist = float(np.linalg.norm(p - base_p0) * 1000)
            dyaw = _yaw_deg(q) - _yaw_deg(base_q0)
            # 归一到 [-180, 180]
            dyaw = (dyaw + 180.0) % 360.0 - 180.0
            return dist, dyaw
        except Exception:  # noqa: BLE001
            return 0.0, 0.0

    # 把视窗相机对准场景（第三人称）
    try:
        import omni.kit.viewport.utility as vp_util

        vp = vp_util.get_active_viewport()
        if vp:
            _log(f"viewport 分辨率 {vp.resolution}")
        cam = vp.get_active_camera() if vp else None
        if cam:
            from pxr import Gf
            import omni.usd as ou

            stage = ou.get_context().get_stage()
            prim = stage.GetPrimAtPath(str(cam))
            if prim and prim.IsValid():
                from pxr import UsdGeom

                xf = UsdGeom.Xformable(prim)
                xf.ClearXformOpOrder()
                xf.AddTranslateOp().Set(Gf.Vec3d(3.2, -1.1, 1.7))
                xf.AddRotateXYZOp().Set(Gf.Vec3f(60.0, 0.0, 55.0))
                _log(f"视窗相机 {cam} → 第三人称 (3.2,-1.1,1.7)")
    except Exception as exc:  # noqa: BLE001
        _log(f"WARN 调整视窗相机失败: {exc}")

    # ---- 回放 ----
    timeline = omni.timeline.get_timeline_interface()
    if not timeline.is_playing():
        timeline.play()
    for _ in range(10):
        app.update()

    arm_t = target_idx[np.asarray(arm_k)]
    arm_pos = np.asarray(arm_k)
    grip_ids = np.asarray(list(grip_targets.keys()), dtype=np.int64)

    # 手臂 drive gain：必须走 drive，否则瞬移会让接触约束每帧失效
    use_drive = args.arm_mode == "drive"
    if use_drive:
        try:
            n = len(arm_t)
            art.set_dof_gains(
                stiffnesses=np.full((1, n), args.arm_stiffness, dtype=np.float32),
                dampings=np.full((1, n), args.arm_damping, dtype=np.float32),
                dof_indices=arm_t,
            )
            _log(
                f"手臂 drive: K={args.arm_stiffness:g} D={args.arm_damping:g}（{n} 个关节）"
            )
        except Exception as exc:  # noqa: BLE001
            _log(f"WARN 设置手臂 gain 失败，退回 teleport: {exc}")
            use_drive = False
    else:
        _log("手臂 teleport（瞬移）：接触约束会失效，物体可能夹不住")

    def apply_arm(values: np.ndarray) -> None:
        if use_drive:
            art.set_dof_position_targets(values, dof_indices=arm_t)
        else:
            art.set_dof_positions(values, dof_indices=arm_t)

    t_wall0 = time.monotonic()
    per_frame = []
    tube_dev_max = 0.0
    max_joint_err = 0.0
    base_drift_max = 0.0
    base_yaw_max = 0.0

    _log("开始回放（关闭窗口即停止）")
    try:
        for i in range(n_start, n_end):
            if not app.is_running():
                break
            row = ep.joint_pos[i][source_idx]
            if np.all(np.isfinite(row)):
                apply_arm(row[arm_pos].astype(np.float32).reshape(1, -1))
                if args.gripper_mode == "drive":
                    for dof, (open_t, close_t) in grip_targets.items():
                        k = [kk for kk in grip_k if int(target_idx[kk]) == dof][0]
                        closed = row[k] < (open_t - 0.25 * 0.0445)
                        tgt = close_t if closed else open_t
                        art.set_dof_position_targets(
                            np.array([[tgt]], dtype=np.float32),
                            dof_indices=np.array([dof], dtype=np.int64),
                        )
                elif args.gripper_mode == "teleport":
                    g_pos = np.asarray(grip_k)
                    art.set_dof_positions(
                        row[g_pos].astype(np.float32).reshape(1, -1),
                        dof_indices=target_idx[g_pos],
                    )

            app.update()

            if i % 20 == 0:
                try:
                    act = np.asarray(
                        art.get_dof_positions(dof_indices=arm_t), dtype=np.float32
                    ).reshape(-1)
                    if np.all(np.isfinite(row)):
                        e = float(np.max(np.abs(act - row[arm_pos].astype(np.float32))))
                        max_joint_err = max(max_joint_err, e)
                except Exception:  # noqa: BLE001
                    pass
                devs = {}
                for label, view in obj_views.items():
                    pos_o, _ = ep.objects[label]
                    if not np.all(np.isfinite(pos_o[i])):
                        continue
                    try:
                        cur = np.asarray(view.get_world_poses()[0]).reshape(-1)[:3]
                        devs[label] = float(np.linalg.norm(cur - pos_o[i]) * 1000)
                    except Exception:  # noqa: BLE001
                        pass
                if "test_tube" in devs:
                    tube_dev_max = max(tube_dev_max, abs(devs["test_tube"]))
                bdist, byaw = _base_drift()
                base_drift_max = max(base_drift_max, abs(bdist))
                base_yaw_max = max(base_yaw_max, abs(byaw))
                gripper_act = {}
                if grip_ids.size:
                    try:
                        ga = np.asarray(
                            art.get_dof_positions(dof_indices=grip_ids), dtype=np.float32
                        ).reshape(-1)
                        for dof, val in zip(grip_ids.tolist(), ga.tolist()):
                            gripper_act[str(dof)] = round(float(val), 4)
                    except Exception:  # noqa: BLE001
                        pass
                per_frame.append(
                    {"frame": i, "t": float(ep.t[i]), **devs,
                     "base_drift_mm": bdist, "base_yaw_deg": byaw,
                     "gripper_actual": gripper_act}
                )
                if i % 200 == 0:
                    d = " ".join(f"{k}={v:.0f}mm" for k, v in devs.items())
                    g = " ".join(f"{v:+.4f}" for v in gripper_act.values())
                    _log(
                        f"  frame={i} t={ep.t[i]:.2f}s {d} "
                        f"| base 漂移={bdist:.1f}mm 偏航={byaw:+.2f}° | 夹爪实际 [{g}]"
                    )

            # 限速到实时（speed 可调）
            if args.speed > 0:
                target_wall = t_wall0 + (i - n_start) * 0.02 / args.speed
                sleep = target_wall - time.monotonic()
                if sleep > 0:
                    time.sleep(sleep)
    finally:
        report = {
            "episode": str(args.episode),
            "frames": len(per_frame),
            "gripper_mode": args.gripper_mode,
            "keep_ros": args.keep_ros,
            "lock_base": args.lock_base,
            "speed": args.speed,
            "joint_max_error_mrad": max_joint_err * 1000,
            "test_tube_max_dev_mm": tube_dev_max,
            "base_drift_max_mm": base_drift_max,
            "base_yaw_max_deg": base_yaw_max,
            "samples": per_frame,
        }
        (out / "replay_gui_report.json").write_text(
            json.dumps(report, indent=2, ensure_ascii=False), encoding="utf-8"
        )
        _log(f"关节 max_err = {max_joint_err*1000:.2f} mrad")
        _log(f"底盘漂移 max = {base_drift_max:.1f} mm，偏航 max = {base_yaw_max:+.2f}°"
             f"（lock_base={args.lock_base}）")
        _log(f"试管最大偏差 = {tube_dev_max:.1f} mm")
        _log(f"报告: {out / 'replay_gui_report.json'}")
        try:
            app.close()
        except Exception:  # noqa: BLE001
            pass

    return 0


if __name__ == "__main__":
    sys.exit(main())
