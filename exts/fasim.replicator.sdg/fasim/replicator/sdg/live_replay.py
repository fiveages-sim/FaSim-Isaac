"""在同一 Isaac 实例里回放 episode —— 不重开 stage、不起新窗口。

与 ``renderer.render_episode`` 的区别
=====================================
``render_episode`` 是**离线渲染**：自己开 ``SimulationApp``、重开 stage、
锁底盘、采样出图。适合批量出数据。

本模块是**在线回放**：复用你已经开着的那个 Isaac（跑编排用的那个），
直接在当前 stage 上驱动关节。用途是「肉眼确认采集数据对不对」：

- 不用再弹一个窗口（你本来就在看那个仿真）
- 不需要重新加载场景（省几十秒）
- 末端回放更直接（当前 stage 就有运控，可以直接发话题）

两种驱动方式
============
``drive="joints"``
    逐帧 ``set_dof_position_targets`` 驱动操作关节。
    **不碰底盘**（轮子/脚轮不驱动）、不碰物体（物体按物理自然响应）。
    与 ``render_episode`` 同一套驱动逻辑，所以验证效果一致。

``drive="ee"``
    把末端位姿逐帧发到运控的 ``/left_target`` / ``/right_target``
    （``geometry_msgs/Pose``），由 OCS2 自己算关节。
    要求仿真里 ``ROS=fiveages`` 变体在跑。

为什么不用 ``render_episode`` 直接跑
===================================
它会 ``ctx.open_stage(...)`` 把当前 stage 换掉 —— 那等于把你正在用的
仿真重置了，运控也会被重载。这里刻意**不碰 stage**。

实现要点
========
- 驱动挂在**主线程**的 ``app.update()`` 循环里，不用 physics step 回调
  （回放是主动推进，不是被动观察）。
- 必须在 Script Editor / 扩展里跑（有 ``omni.kit.app``），不能独立进程。
- 循环里每帧 ``app.update()`` 一次 —— 与 ``render_episode`` 的时间基准
  一致（仿真步长由 kit 决定）。
"""

from __future__ import annotations

from pathlib import Path
from typing import Any, Callable

import numpy as np

_STATE: dict[str, Any] = {
    "playing": False,
    "paused": False,
    "frame": 0,
    "total": 0,
    "episode": None,
    "cancel": False,
}


def _log(msg: str) -> None:
    print(f"[live] {msg}", flush=True)


def is_playing() -> bool:
    return bool(_STATE["playing"])


def status() -> dict:
    ep = _STATE.get("episode")
    ctx = _STATE.get("ctx") or {}
    idx = int(_STATE.get("frame", 0))
    t = 0.0
    if ep is not None and 0 <= idx < len(ep):
        t = float(ep.t[idx])
    return {
        "playing": _STATE["playing"],
        "paused": _STATE["paused"],
        "frame": idx,
        "total": int(ctx.get("n_end") or _STATE.get("total") or 0),
        "t": t,
        "app_dt_ms": round(float(ctx.get("app_dt", 0.0)) * 1000, 2),
        "frames_done": int(ctx.get("frames_done", 0)),
        "last_result": _STATE.get("last_result"),
    }


def pause() -> None:
    _STATE["paused"] = True
    _log("已暂停")


def resume() -> None:
    _STATE["paused"] = False
    _log("继续")


def stop() -> None:
    _STATE["cancel"] = True
    _log("收到停止请求（当前帧结束后退出）")


def _resolve_views(robot_cfg) -> tuple[Any, dict, dict]:
    """拿 articulation / 操作关节映射 / 夹爪规格。"""
    from isaacsim.core.experimental.prims import Articulation

    from .episode import build_dof_mapping, skipped_dof_names

    art = Articulation(robot_cfg.articulation_prim)
    dofs = list(art.dof_names)
    return art, dofs, {}


def play_episode(
    episode_path: str | Path,
    robot_key: str = "unitree_g1d",
    drive: str = "joints",
    speed: float = 1.0,
    start: int = 0,
    end: int | None = None,
    loop: bool = False,
    on_frame: Callable[[int, dict], None] | None = None,
    left_topic: str = "/left_target",
    right_topic: str = "/right_target",
    pause_cm: bool = True,
    home_frames: int = 60,
    waist_gain: str = "usd",
    joint_api: str = "target",
    reset_objects: bool = True,
    task_key: str = "sinopec_test_tube",
) -> dict:
    """在当前 Isaac 实例里**非阻塞**回放 —— 只装载，靠 update 推进。

    **不在这里跑循环**：``play_episode`` 只是把 episode 装进 _STATE 并
    注册一个 update 回调，然后立刻返回。真正的逐帧驱动发生在
    ``_on_update`` 里（每次 ``app.update()`` 推一帧）。

    为什么必须这样：如果在 socket 回调 / Script Editor 里跑 for 循环调
    ``app.update()``，会重入 Kit 主循环，表现是画面卡死或崩溃。

    Args:
        episode_path: episode npz（``sdg.sh pack`` 或 ``trim`` 的产物）。
        robot_key: 机器人配置 key。
        drive: ``"joints"`` 或 ``"ee"``。
        speed: 播放倍速（仅 ``drive="ee"`` 时用于调发送频率；
            ``joints`` 的节奏由 app.update 决定）。
        start/end: 帧区间。
        loop: 播完循环。
        on_frame: 每帧回调。
        left_topic / right_topic: ``drive="ee"`` 时的话题。

    Returns:
        ``{"ok": True, "frames": N, "drive": ...}``
    """
    import omni.kit.app
    import omni.timeline
    import omni.usd

    from .config_io import load_robot_config
    from .episode import Episode, build_dof_mapping, skipped_dof_names

    app = omni.kit.app.get_app()

    if _STATE["playing"]:
        return {"ok": False, "error": "已在回放中，先 sdg.sh live stop"}

    st = omni.usd.get_context().get_stage()
    if st is None:
        return {"ok": False, "error": "当前没有打开的 stage —— 先加载场景"}

    episode = Episode.load(episode_path)
    n_total = len(episode)
    n_start = max(0, int(start))
    # CLI 约定 end=0 表示"到末尾"（与 sdg.sh 的其他命令一致）。
    # 这里必须显式处理，否则 min(n_total, 0) = 0 会得到空区间。
    end_i = int(end or 0)
    n_end = n_total if end_i <= 0 else min(n_total, end_i)
    if n_end <= n_start:
        return {
            "ok": False,
            "error": f"空区间 start={n_start} end={n_end}（episode 共 {n_total} 帧）",
        }

    robot = load_robot_config(robot_key)
    if not st.GetPrimAtPath(robot.articulation_prim):
        return {
            "ok": False,
            "error": f"当前 stage 没有 {robot.articulation_prim} —— 先加载场景",
        }

    # ---- 准备驱动目标 ----
    frame_dt = 0.05
    if len(episode.t) >= 2:
        d = np.diff(np.asarray(episode.t, dtype=np.float64))
        d = d[np.isfinite(d) & (d > 0)]
        if d.size:
            frame_dt = float(np.median(d))

    ctx: dict[str, Any] = {
        "drive": drive,
        "episode": episode,
        "n_start": n_start,
        "n_end": n_end,
        "loop": loop,
        "on_frame": on_frame,
        "frame_dt": frame_dt,
        "speed": speed,
        "errs": [],
        "max_err": 0.0,
        "frames_done": 0,
        "pause_cm": bool(pause_cm),
        "waist_gain": waist_gain,
        "joint_api": str(joint_api or "target"),
        "robot_key": robot_key,
        "task_key": task_key,
        "reset_objects": bool(reset_objects),
        "cm_done": True,  # _pause_controllers 会改成 False（异步流程）
    }

    if drive == "ee":
        if not np.isfinite(np.asarray(episode.left_pos)).all() or \
           not np.isfinite(np.asarray(episode.right_pos)).all():
            return {
                "ok": False,
                "error": "episode 没有末端位姿（left_pos/right_pos 含 NaN）。"
                         "采集要选 ee：sdg.sh capture --what joint,ee",
            }
        try:
            import rclpy
            from geometry_msgs.msg import Pose
        except ImportError:
            return {"ok": False, "error": "没有 rclpy —— 需要 source 过 ROS 的 Isaac"}
        if not rclpy.ok():
            rclpy.init()
        node = rclpy.create_node("sdg_live_ee")
        ctx.update(
            node=node,
            rclpy=rclpy,
            pub_l=node.create_publisher(Pose, left_topic, 1),
            pub_r=node.create_publisher(Pose, right_topic, 1),
            mk_pose=_make_pose_factory(Pose),
            last_emit=0.0,
        )
        _log(f"末端驱动 → {left_topic} / {right_topic}，{1/(frame_dt/speed):.1f} Hz")
    else:
        from isaacsim.core.experimental.prims import Articulation

        art = Articulation(robot.articulation_prim)
        dofs = list(art.dof_names)
        target_idx, source_idx, missing = build_dof_mapping(dofs, episode.joint_names)

        if len(source_idx):
            ok = np.isfinite(episode.joint_pos[:, source_idx]).any(axis=0)
            dead = [episode.joint_names[c] for c, k in zip(source_idx, ok) if not k]
            if dead:
                _log(f"跳过整列 NaN: {dead}")
                target_idx = target_idx[ok]
                source_idx = source_idx[ok]

        if len(target_idx):
            non_manip = set(skipped_dof_names(dofs))
            keep = [i for i, d in enumerate(target_idx) if dofs[int(d)] not in non_manip]
            if len(keep) != len(target_idx):
                dropped = [dofs[int(target_idx[i])] for i in range(len(target_idx))
                           if i not in keep]
                _log(f"剔除底盘 {len(dropped)} 个（不驱动）: {dropped}")
                target_idx = target_idx[keep]
                source_idx = source_idx[keep]

        if len(target_idx) == 0:
            return {"ok": False, "error": "没有可驱动关节 —— episode 与场景不匹配"}
        if missing:
            _log(f"WARN 找不到的关节: {missing}")

        grip_specs = []
        try:
            from .renderer import _build_gripper_switches

            grip_specs = _build_gripper_switches(
                dofs, target_idx, source_idx, list(episode.joint_names),
                episode.joint_pos, art,
            )
            if grip_specs:
                gd = {s["dof"] for s in grip_specs}
                keep = [i for i, d in enumerate(target_idx) if int(d) not in gd]
                target_idx = target_idx[keep]
                source_idx = source_idx[keep]
        except Exception as exc:  # noqa: BLE001
            _log(f"WARN 夹爪开关量失败（按位置回放）: {exc}")

        # **剔除 mimic 关节**（如 Dex1 的 *_gripper_joint1，限位 [0,0]）。
        # _build_gripper_switches 会跳过它们（不进 grip_specs），于是上面的
        # gd 里没有它们 → 仍留在位置驱动列表里。
        # 但它们挂了 PhysxMimicJointAPI，由主关节带动；再给它们设 drive
        # target 会和 mimic 打架，扰动整条运动链（实测多驱动 4 个 mimic）。
        try:
            lims = art.get_dof_limits()
            lo_all = np.asarray(lims[0]).reshape(-1)
            hi_all = np.asarray(lims[1]).reshape(-1)
            keep2 = [
                i for i, d in enumerate(target_idx)
                if float(hi_all[int(d)] - lo_all[int(d)]) > 1e-6
            ]
            dropped = [dofs[int(target_idx[i])] for i in range(len(target_idx))
                       if i not in keep2]
            if dropped:
                _log(f"剔除 mimic 关节 {len(dropped)} 个（由 mimic API 跟随）: {dropped}")
                target_idx = target_idx[keep2]
                source_idx = source_idx[keep2]
        except Exception as exc:  # noqa: BLE001
            _log(f"WARN 剔除 mimic 失败: {exc}")

        ctx.update(art=art, target_idx=target_idx, source_idx=source_idx,
                   grip_specs=grip_specs,
                   target_names=[dofs[int(d)] for d in target_idx])
        # 手臂 drive gain —— 与 render_episode 一致。
        # 不设的话用资产默认值（Dex1 的 gain 是给夹爪调的），手臂追不上
        # 目标，实测关节误差 268 mrad。
        #
        # **腰部（prismatic 升降）要单独处理**：USD 里是
        #   LZ_mt_Joint: stiffness=60000 damping=6000 maxForce=5000
        # 和手臂的 revolute 特性完全不同。统一设 100000 可能让它过硬/过阻尼，
        # 实测编排里腰部上抬 8.1cm，回放幅度明显偏小。
        try:
            arm_sel = target_idx
            waist_sel = np.array([], dtype=np.int64)
            if waist_gain and waist_gain.lower() != "same":
                wset = {"LZ_mt_Joint", "LZ_it_Joint", "waist_pitch_joint", "waist_mount_joint"}
                w_mask = np.array([dofs[int(d)] in wset for d in target_idx])
                waist_sel = target_idx[w_mask]
                arm_sel = target_idx[~w_mask]
                if len(waist_sel):
                    maxf = None
                    if waist_gain.lower() == "usd":
                        k, d = 60000.0, 6000.0  # USD 原值
                        _log(f"腰部用 USD 原值 K={k:g} D={d:g}（{len(waist_sel)} 个）")
                    else:
                        parts = [float(x) for x in waist_gain.split(",")]
                        k = parts[0]
                        d = parts[1] if len(parts) > 1 else 6000.0
                        maxf = parts[2] if len(parts) > 2 else None
                        _log(f"腰部自定义 K={k:g} D={d:g}"
                             + (f" maxForce={maxf:g}" if maxf else "")
                             + f"（{len(waist_sel)} 个）")
                    art.set_dof_gains(
                        stiffnesses=np.full((1, len(waist_sel)), k, dtype=np.float32),
                        dampings=np.full((1, len(waist_sel)), d, dtype=np.float32),
                        dof_indices=waist_sel,
                    )
                    if maxf is not None:
                        try:
                            art.set_dof_max_efforts(
                                np.full((1, len(waist_sel)), maxf, dtype=np.float32),
                                dof_indices=waist_sel,
                            )
                        except Exception as exc2:  # noqa: BLE001
                            _log(f"WARN 设腰部 maxForce 失败: {exc2}")

            if len(arm_sel):
                n = len(arm_sel)
                k, d = 100000.0, 10000.0
                art.set_dof_gains(
                    stiffnesses=np.full((1, n), k, dtype=np.float32),
                    dampings=np.full((1, n), d, dtype=np.float32),
                    dof_indices=arm_sel,
                )
                _log(f"手臂 drive gain K={k:g} D={d:g}（{n} 个关节）")
        except Exception as exc:  # noqa: BLE001
            _log(f"WARN 设 drive gain 失败: {exc}")

        # **必须让运控交出控制权**。仿真里 in-process CM 每 tick 往同一批
        # 关节写位置，会和我们的目标互相覆盖 —— 表现是动作时快时慢、关节
        # 偏几厘米。层2 离线回放没有 CM 抢，所以只要 3 mrad。
        #
        # 注意：这里是**异步发起**，由 _on_update 每帧推进（不能阻塞等，
        # 见 _pause_controllers 的 docstring）。
        if pause_cm:
            _pause_controllers(ctx)
        else:
            ctx["cm_done"] = True

        _log(f"关节驱动 {len(target_idx)} 个，帧 {n_start}..{n_end}")

        # 记录**回放前**的关节位置，退出时恢复（否则运控接管时会从陌生
        # 姿态开始规划，看起来像"运控坏了"）。
        try:
            q0 = np.asarray(art.get_dof_positions(), dtype=np.float32).reshape(-1)
            ctx["initial_dof"] = q0.copy()
            _log(f"已记录回放前姿态（{len(q0)} DOF），退出时恢复")
        except Exception as exc:  # noqa: BLE001
            _log(f"WARN 记录初始姿态失败: {exc}")

        # 物体复位：回放开始时物体可能停在上次跑完的位置（被撞歪/抓走），
        # 手按关节角伸过去就抓空。放回 episode 第 0 帧的位姿后交给物理。
        if ctx.get("reset_objects"):
            nobj = _reset_objects(ctx)
            if nobj:
                _log(f"已复位 {nobj} 个交互物体")
            else:
                _log(
                    "WARN 没有物体位姿可复位（episode 缺 obj/*_pos）。"
                    "这会导致抓取错位 —— 采集时请选 object："
                    "sdg.sh capture --what joint,ee,object"
                )

        # 归位：从当前姿态平滑过渡到第 0 帧，避免 drive 猛冲把物体撞飞。
        # 实测不归位时：右手从远处瞬间追到第 0 帧，扫过试管架把它带起来。
        try:
            first = episode.joint_pos[n_start][source_idx]
            if np.all(np.isfinite(first)):
                ctx["home_target"] = first.astype(np.float32)
                ctx["homing"] = bool(home_frames > 0)
                if ctx["homing"]:
                    ctx["home_task"] = True
                    ctx["home_total"] = max(1, int(home_frames))
                    ctx["home_step"] = 0
                    _log(f"归位过渡 {home_frames} 帧 → 第 {n_start} 帧姿态")
        except Exception as exc:  # noqa: BLE001
            _log(f"WARN 归位目标不可用: {exc}")
            ctx["homing"] = False

    timeline = omni.timeline.get_timeline_interface()
    if not timeline.is_playing():
        timeline.play()

    # 记录起始仿真时间，供 _ensure_app_dt 差分（第一次回调时 last=None
    # 会跳过一帧，这里先埋一个基准）
    ctx["_last_sim_time"] = None

    _STATE.update(playing=True, paused=False, frame=n_start, total=n_end,
                  episode=episode, cancel=False, ctx=ctx, i=n_start,
                  since_frame=0.0)
    ctx["since_frame"] = 0.0

    if not _install_update_hook():
        _STATE.update(playing=False)
        return {"ok": False, "error": "注册 update 回调失败"}

    return {
        "ok": True,
        "drive": drive,
        "frames": n_end - n_start,
        "start": n_start,
        "end": n_end,
        "driven_joints": int(len(ctx.get("target_idx", []))),
        "frame_dt": frame_dt,
    }



# ---------------------------------------------------------------- update 驱动


def _pause_controllers(ctx: dict) -> list[str]:
    """回放关节前把运控控制器置为 inactive，避免它覆盖我们的目标。

    为什么必须做
    ============
    仿真里 ``ROS=fiveages`` 变体起的 in-process ControllerManager 每 tick
    都会往同一批关节写位置。我的 ``set_dof_position_targets`` 立刻被它
    覆盖，结果是两个控制器互相拉扯：

    - 动作时快时慢（OCS2 重新规划又被打断）
    - 关节偏几厘米（最终位置是两边的折中；实测 325 mrad，而层2 离线
      回放只要 3 mrad —— 差别就在有没有 CM 抢控制权）

    **绝对不能阻塞等待服务响应**
    ============================
    in-process CM 的 ``update()`` 由 Isaac 主循环驱动。我们在主循环的
    回调里 ``spin_until_future_complete`` 等它，而它要等主循环推进才能
    处理请求 —— 必然超时（实测 "Switch controller timed out after 3
    seconds!"）。

    所以：发请求 → 立刻返回；由 ``_on_update`` 每帧 ``spin_once`` 推进，
    拿到结果后再真正开始回放。
    """
    ctx["cm_done"] = False
    ctx["cm_ok"] = False
    try:
        import rclpy
        from controller_manager_msgs.srv import ListControllers, SwitchController
    except ImportError:
        _log("WARN 没有 controller_manager_msgs，无法暂停运控 —— 回放精度会受影响")
        ctx["cm_done"] = True
        return []

    try:
        if not rclpy.ok():
            rclpy.init()
        node = ctx.get("cm_node")
        if node is None:
            node = rclpy.create_node("sdg_live_pause_cm")
            ctx["cm_node"] = node

        lis = node.create_client(ListControllers, "controller_manager/list_controllers")
        if not lis.service_is_ready():
            # 非阻塞：由 _poll_cm 每帧重试（wait_for_service 会阻塞主循环）
            ctx["cm_phase"] = "wait_list"
            ctx["cm_client"] = lis
            ctx["cm_wait_list"] = 0
            _log("等待 controller_manager 服务（异步）…")
            return []

        fut = lis.call_async(ListControllers.Request())
        ctx["cm_phase"] = "list"
        ctx["cm_future"] = fut
        ctx["cm_client"] = lis
        _log("正在查询运控控制器列表（异步）…")
        return []
    except Exception as exc:  # noqa: BLE001
        _log(f"WARN 暂停运控失败: {exc}")
        ctx["cm_done"] = True
        return []


def _poll_cm(ctx: dict) -> bool:
    """每帧调用：推进 CM 服务请求。返回 True 表示处理完毕。"""
    if ctx.get("cm_done"):
        return True
    node = ctx.get("cm_node")
    if node is None:
        ctx["cm_done"] = True
        return True

    try:
        import rclpy

        rclpy.spin_once(node, timeout_sec=0.0)
    except Exception:  # noqa: BLE001
        pass

    # 阶段：等服务就绪（非阻塞重试）
    if ctx.get("cm_phase") == "wait_list":
        lis = ctx.get("cm_client")
        if lis is not None and lis.service_is_ready():
            try:
                from controller_manager_msgs.srv import ListControllers

                ctx["cm_future"] = lis.call_async(ListControllers.Request())
                ctx["cm_phase"] = "list"
                _log("controller_manager 已就绪，查询控制器列表")
            except Exception as exc:  # noqa: BLE001
                _log(f"WARN 构造 list 请求失败: {exc}")
                ctx["cm_done"] = True
                return True
        else:
            n = ctx.get("cm_wait_list", 0) + 1
            ctx["cm_wait_list"] = n
            if n > 600:  # 约 600 帧 ≈ 10s
                _log("WARN 等不到 controller_manager，跳过暂停运控")
                ctx["cm_done"] = True
                return True
        return False

    fut = ctx.get("cm_future")
    if fut is None:
        ctx["cm_done"] = True
        return True
    if not fut.done():
        return False

    phase = ctx.get("cm_phase")
    try:
        res = fut.result()
    except Exception as exc:  # noqa: BLE001
        _log(f"WARN CM 请求失败: {exc}")
        ctx["cm_done"] = True
        return True

    if phase == "list":
        active = [c.name for c in (res.controller if res else []) if c.state == "active"]
        if not active:
            _log("运控里没有 active 控制器，直接回放")
            ctx["cm_done"] = True
            return True
        try:
            from controller_manager_msgs.srv import SwitchController

            sw = node.create_client(
                SwitchController, "controller_manager/switch_controller"
            )
            if not sw.service_is_ready():
                # 非阻塞：下一帧再试（wait_for_service 会阻塞主循环 → 死锁）
                if not ctx.get("cm_wait_sw"):
                    ctx["cm_wait_sw"] = 0
                ctx["cm_wait_sw"] += 1
                if ctx["cm_wait_sw"] > 200:  # 约 200 帧 ≈ 3s
                    _log("WARN 等不到 switch_controller 服务，跳过暂停")
                    ctx["cm_done"] = True
                    return True
                return False
            req = SwitchController.Request()
            req.deactivate_controllers = active
            req.activate_controllers = []
            req.strictness = SwitchController.Request.BEST_EFFORT
            ctx["cm_active"] = active
            ctx["cm_future"] = sw.call_async(req)
            ctx["cm_phase"] = "switch"
            _log(f"请求停用 {len(active)} 个控制器: {active}")
        except Exception as exc:  # noqa: BLE001
            _log(f"WARN 构造 switch 请求失败: {exc}")
            ctx["cm_done"] = True
        return False

    if phase == "switch":
        ok = bool(res is not None and getattr(res, "ok", False))
        active = ctx.get("cm_active") or []
        if ok:
            _log(f"已暂停运控控制器（{len(active)} 个）: {active}")
            ctx["paused_controllers"] = active
        else:
            _log(f"WARN 暂停控制器失败: {getattr(res, 'message', res)}")
        ctx["cm_done"] = True
        return True

    ctx["cm_done"] = True
    return True


def _restore_initial_pose(ctx: dict) -> None:
    """把关节送回回放前的位置（在运控重新接管之前）。

    用 ``set_dof_positions``（teleport）而不是 drive：这里只要"摆回去"，
    不需要物理过渡；drive 要几百帧才收敛，而我们已经退订了 update 回调。
    """
    art = ctx.get("art")
    target_idx = ctx.get("target_idx")
    initial = ctx.get("initial_dof")
    if art is None or target_idx is None or initial is None:
        return
    try:
        vals = np.asarray(initial, dtype=np.float32)[target_idx].reshape(1, -1)
        art.set_dof_positions(vals, dof_indices=target_idx)
        _log(f"已把 {len(target_idx)} 个关节恢复到回放前位置")
    except Exception as exc:  # noqa: BLE001
        _log(f"WARN 恢复初始姿态失败: {exc}")


def _resume_controllers(ctx: dict) -> None:
    """回放结束后把之前暂停的控制器恢复。

    同样**不能阻塞**：这里只是发起请求，由后续帧推进。
    若回放已结束、不会再进 _on_update，则用一个短轮询（此时不在回调里，
    主循环仍能推进 CM）。
    """
    names = ctx.get("paused_controllers") or []
    if not names:
        return
    try:
        import rclpy
        from controller_manager_msgs.srv import SwitchController

        node = ctx.get("cm_node")
        if node is None:
            return
        sw = node.create_client(SwitchController, "controller_manager/switch_controller")
        if not sw.service_is_ready() and not sw.wait_for_service(timeout_sec=1.0):
            _log("WARN 恢复时找不到 switch_controller")
            return
        req = SwitchController.Request()
        req.activate_controllers = names
        req.deactivate_controllers = []
        req.strictness = SwitchController.Request.BEST_EFFORT
        fut = sw.call_async(req)
        # 短轮询：不 spin_until，但给 CM 几帧机会处理
        import time

        t0 = time.monotonic()
        while not fut.done() and time.monotonic() - t0 < 2.0:
            try:
                rclpy.spin_once(node, timeout_sec=0.05)
            except Exception:  # noqa: BLE001
                break
            time.sleep(0.01)
        if fut.done() and fut.result() is not None and fut.result().ok:
            _log(f"已恢复运控控制器: {names}")
        else:
            _log(f"WARN 恢复控制器失败: {fut.result() if fut.done() else '超时'}")
    except Exception as exc:  # noqa: BLE001
        _log(f"WARN 恢复运控失败: {exc}")
    finally:
        ctx["paused_controllers"] = []


def _make_pose_factory(Pose):
    """返回 p,q → geometry_msgs/Pose 的构造函数。"""

    def mk(p, q) -> Any:
        m = Pose()
        m.position.x, m.position.y, m.position.z = (float(x) for x in p[:3])
        m.orientation.w, m.orientation.x, m.orientation.y, m.orientation.z = (
            float(x) for x in q[:4]
        )
        return m

    return mk


_HOOK: dict = {"sub": None}


def _install_update_hook() -> bool:
    """注册 app update 回调 —— 每帧推一帧回放。"""
    if _HOOK["sub"] is not None:
        return True
    try:
        import omni.kit.app

        app = omni.kit.app.get_app()
        _HOOK["sub"] = app.get_update_event_stream().create_subscription_to_pop(
            _on_update, name="fasim.sdg.live_replay"
        )
        return True
    except Exception as exc:  # noqa: BLE001
        _log(f"WARN 注册 update 回调失败: {exc}")
        return False


def _uninstall_update_hook() -> None:
    sub = _HOOK.get("sub")
    _HOOK["sub"] = None
    if sub is not None:
        try:
            sub.unsubscribe()
        except Exception:  # noqa: BLE001
            pass


def _on_update(_event) -> None:
    """推进一步。**必须极快返回**，否则拖慢整个 Kit。

    时间基准
    ========
    录制的帧间隔（如 50ms）往往**大于**一次 ``app.update()`` 推进的
    仿真时间（16.67ms）。若每帧只 update 一次，回放会快 3 倍 —— 动作在
    屏幕上"跳"，而不是连续播放。

    做法：本回调**自己就是一次 update**，所以用"欠账"计数：目标是让
    episode 的 t 轴与仿真时间对齐。每进一次回调，仿真推进 app_dt；
    当累计推进量够一帧（frame_dt）时才切到下一帧，否则重复当前帧的
    目标（保持不动，让物理继续走）。

    **绝不能在这里 for 循环调 app.update()** —— 会重入 Kit 主循环。
    """
    if not _STATE["playing"]:
        return
    if _STATE["cancel"]:
        _finish()
        return
    if _STATE["paused"]:
        return

    ctx = _STATE.get("ctx") or {}
    if not ctx:
        _finish()
        return

    # 阶段 0：等运控控制器停用完成（异步，不阻塞主循环）
    if not ctx.get("cm_done", True):
        if not _poll_cm(ctx):
            return
        # 停用完成（成功或失败）：清掉上一帧目标，从头开始
        ctx["since_frame"] = 0.0
        ctx["_last_sim_time"] = None
        if ctx.get("paused_controllers"):
            _log("运控已让出控制权，开始回放")
        else:
            _log("运控仍活跃 —— 关节可能被抢占（精度会下降）")

    app_dt = _ensure_app_dt(ctx)
    frame_dt = ctx.get("frame_dt", 0.05)
    if app_dt <= 0:
        # 首帧还不知道步长；先设第 0 帧目标，下一帧再开始累积
        if ctx.get("since_frame", 0.0) == 0.0:
            try:
                if ctx["drive"] == "ee":
                    _step_ee(ctx, _STATE["i"])
                else:
                    _step_joints(ctx, _STATE["i"])
                ctx["since_frame"] = 0.001
            except Exception:  # noqa: BLE001
                pass
        return

    # 阶段 1：归位过渡（从当前姿态线性插值到第 0 帧）
    if ctx.get("homing"):
        _step_homing(ctx)
        ctx["home_step"] = ctx.get("home_step", 0) + 1
        if ctx["home_step"] >= ctx["home_total"]:
            ctx["homing"] = False
            ctx["since_frame"] = 0.0
            _log("归位完成，开始回放")
        return

    # 本回调已经推进了 app_dt 的仿真时间
    ctx["since_frame"] = ctx.get("since_frame", 0.0) + app_dt
    advance = ctx["since_frame"] >= frame_dt - app_dt * 0.5

    i = _STATE["i"]
    n_end = ctx["n_end"]
    episode = ctx["episode"]

    try:
        if advance:
            ctx["since_frame"] = 0.0
            if ctx["drive"] == "ee":
                _step_ee(ctx, i)
            else:
                _step_joints(ctx, i)
        else:
            # 还没到下一帧：重发当前目标，保持姿态（不要跳帧）
            if ctx["drive"] == "joints":
                _hold_joints(ctx, i)
    except Exception as exc:  # noqa: BLE001
        import traceback

        _log(f"回放出错（第 {i} 帧）: {exc!r}")
        traceback.print_exc()
        _finish()
        return

    if not advance:
        return

    _STATE["frame"] = i
    ctx["frames_done"] = ctx.get("frames_done", 0) + 1

    # 首帧立刻打印，方便确认回放真的开始了
    if ctx["frames_done"] == 1:
        _log(f"回放首帧已执行: frame={i} app_dt={app_dt*1000:.2f}ms "
             f"frame_dt={frame_dt*1000:.1f}ms")
    cb = ctx.get("on_frame")
    if cb is not None:
        try:
            cb(i, {"t": float(episode.t[i]), "frame": i})
        except Exception:  # noqa: BLE001
            pass
    fd = ctx["frames_done"]
    if fd % 25 == 0:
        me = ctx.get("max_err", 0.0) * 1000
        sim_t = float(tl_now(ctx))
        _log(f"  {i}/{n_end}  sim={sim_t:.2f}s  max_err={me:.1f}mrad  "
             f"(已播 {fd} 帧)")

    i += 1
    if i >= n_end:
        if ctx.get("loop"):
            _STATE["i"] = ctx["n_start"]
            _log("循环播放…")
            return
        _finish()
        return
    _STATE["i"] = i


def _hold_joints(ctx: dict, i: int) -> None:
    """重复当前帧目标（帧间隔 > 单次 update 时用，避免跳帧）。"""
    art = ctx["art"]
    source_idx = ctx["source_idx"]
    target_idx = ctx["target_idx"]
    episode = ctx["episode"]
    row = episode.joint_pos[i][source_idx]
    if np.all(np.isfinite(row)):
        art.set_dof_position_targets(
            row.astype(np.float32).reshape(1, -1), dof_indices=target_idx
        )
    for spec in ctx.get("grip_specs", []):
        closed = bool(spec["closed"][min(i, len(spec["closed"]) - 1)])
        tgt = spec["close_target"] if closed else spec["open_target"]
        art.set_dof_position_targets(
            np.array([[tgt]], dtype=np.float32),
            dof_indices=np.array([spec["dof"]], dtype=np.int64),
        )


def _ensure_app_dt(ctx: dict) -> float:
    """从相邻两次回调的 timeline 时间差推算 app_dt（**不主动 update**）。

    之前在回调里调 ``app.update()`` 测步长 —— 那是**重入** Kit 主循环，
    会抛异常让回放立刻结束（症状：几秒就停、机器人不动）。

    改为被动测量：记录上次回调时的仿真时间，用差分算步长。
    第一次回调时还不知道，返回 None 让调用方跳过这一帧。
    """
    import omni.timeline

    tl = omni.timeline.get_timeline_interface()
    now = float(tl.get_current_time())
    last = ctx.get("_last_sim_time")
    ctx["_last_sim_time"] = now

    if ctx.get("app_dt"):
        return float(ctx["app_dt"])
    if last is None:
        return 0.0  # 首帧，下一次才知道步长
    dt = now - last
    if dt <= 0:
        return 0.0
    ctx["app_dt"] = dt
    _log(f"实测每次 update 推进 {dt*1000:.2f} ms 仿真时间")
    return dt


def _step_homing(ctx: dict) -> None:
    """归位过渡：从记录的回放前姿态线性插值到第 0 帧姿态。

    为什么不能直接跳：drive 会把"位置差"变成力矩。若当前手在远处、第 0
    帧在试管旁，瞬间的目标差会让手臂高速扫过去 —— 实测会把试管架带起来。
    """
    art = ctx["art"]
    target_idx = ctx["target_idx"]
    start = ctx.get("initial_dof")
    goal = ctx.get("home_target")
    total = max(1, int(ctx.get("home_total", 60)))
    step = min(int(ctx.get("home_step", 0)), total - 1)
    if start is None or goal is None:
        ctx["homing"] = False
        return

    # 只对要驱动的关节插值
    s = np.asarray(start, dtype=np.float32)[target_idx]
    g = np.asarray(goal, dtype=np.float32)
    ratio = (step + 1) / total
    # 缓入缓出，避免起止速度突变
    ratio = 0.5 - 0.5 * np.cos(np.pi * ratio)
    val = (s + (g - s) * ratio).astype(np.float32).reshape(1, -1)
    art.set_dof_position_targets(val, dof_indices=target_idx)

    for spec in ctx.get("grip_specs", []):
        closed = bool(spec["closed"][min(ctx["n_start"], len(spec["closed"]) - 1)])
        tgt = spec["close_target"] if closed else spec["open_target"]
        art.set_dof_position_targets(
            np.array([[tgt]], dtype=np.float32),
            dof_indices=np.array([spec["dof"]], dtype=np.int64),
        )


def tl_now(ctx: dict) -> float:
    """当前仿真时间（用于日志）。"""
    try:
        import omni.timeline

        return float(omni.timeline.get_timeline_interface().get_current_time())
    except Exception:  # noqa: BLE001
        return 0.0


def _reset_objects(ctx: dict) -> int:
    """把交互物体复位到 episode 记录的**初始**位姿。

    为什么必须做
    ============
    回放靠物理自然响应（不逐帧摆物体）。但回放开始时物体停在「上一次
    跑完的位置」—— 可能被撞歪、被抓走。手按关节角伸过去自然抓空，
    表现为「抓取错位」。

    正确做法：回放前把物体放回 episode 第 0 帧的位姿（录制起点），
    之后完全交给物理。

    注意：episode 必须有 ``obj/<label>_pos/quat``（采集时选了 object）。
    没有就跳过并提示。
    """
    import omni.usd
    from pxr import UsdPhysics

    from .config_io import load_task_config

    episode = ctx["episode"]
    robot_key = ctx.get("robot_key", "unitree_g1d")
    task_key = ctx.get("task_key", "sinopec_test_tube")
    try:
        task = load_task_config(robot_key, task_key)
        prim_map = dict(getattr(task, "tracked_prims", None) or {})
    except Exception as exc:  # noqa: BLE001
        _log(f"WARN 读任务配置失败，跳过物体复位: {exc}")
        return 0

    st = omni.usd.get_context().get_stage()
    n = 0
    for label, (pos, quat) in (episode.objects or {}).items():
        prim_path = prim_map.get(label)
        if not prim_path:
            continue
        p0 = np.asarray(pos[0], dtype=np.float64)
        q0 = np.asarray(quat[0], dtype=np.float64)
        if not np.all(np.isfinite(p0)) or not np.all(np.isfinite(q0)):
            continue
        prim = st.GetPrimAtPath(str(prim_path))
        if not prim or not prim.IsValid():
            _log(f"WARN 物体 {label} 的 prim 不存在: {prim_path}")
            continue
        try:
            # 用 USD xform 直接写（回放还没开始，物理未接管这个刚体时最可靠）
            from pxr import Gf, UsdGeom

            xf = UsdGeom.Xformable(prim)
            xf.ClearXformOpOrder()
            xf.AddTranslateOp().Set(Gf.Vec3d(*[float(x) for x in p0[:3]]))
            xf.AddOrientOp().Set(
                Gf.Quatf(float(q0[0]), float(q0[1]), float(q0[2]), float(q0[3]))
            )
            _log(f"物体复位 {label} → {np.round(p0, 3)}")
            n += 1
        except Exception as exc:  # noqa: BLE001
            _log(f"WARN 复位物体 {label} 失败: {exc}")
    return n


def _apply_joint_target(ctx: dict, values: np.ndarray, frame: int) -> None:
    """按 ctx["joint_api"] 驱动关节。

    ``target``（默认）: ``set_dof_position_targets`` —— 走 drive 产生力，
        和运控/层2 一致，物理接触正确。缺点是重负载关节（腰部升降）有静差。
    ``teleport``: ``set_dof_positions`` —— 直接设位置，无静差、逐帧精确，
        但会破坏接触（物体不会被自然带动）。
        用途：验证「数据本身对不对」。若 teleport 下腰部幅度正常，
        说明数据没问题，是 drive 跟不上。
    """
    art = ctx["art"]
    target_idx = ctx["target_idx"]
    if ctx.get("joint_api") == "teleport":
        art.set_dof_positions(values, dof_indices=target_idx)
        # **必须清速度**：瞬移会让引擎算出巨大隐含速度，下一帧物理按那个
        # 速度外推 → 全身高频颤抖（用户实测「不停抖动」）。
        try:
            art.set_dof_velocities(
                np.zeros_like(values, dtype=np.float32), dof_indices=target_idx
            )
        except Exception:  # noqa: BLE001
            pass
    else:
        art.set_dof_position_targets(values, dof_indices=target_idx)


def _step_joints(ctx: dict, i: int) -> None:
    art = ctx["art"]
    source_idx = ctx["source_idx"]
    target_idx = ctx["target_idx"]
    episode = ctx["episode"]

    # 1) 先校验**上一帧**的目标 vs 当前 actual。
    #    不能"设完 target 立刻读 actual" —— 此时物理还没步进，读到的是
    #    上一帧的值，会得到假误差（实测恒定 268 mrad，加 gain 完全没变化
    #    就是因为这个）。
    prev = ctx.get("prev_values")
    if prev is not None:
        try:
            act = np.asarray(
                art.get_dof_positions(dof_indices=target_idx), dtype=np.float32
            ).reshape(-1)
            e = np.abs(act - prev)
            ctx.setdefault("errs", []).append(float(np.max(e)))
            ctx["max_err"] = max(ctx.get("max_err", 0.0), float(np.max(e)))
            # 按关节累计最大误差，便于定位是哪个关节跟不上
            per = ctx.setdefault("err_per_joint", {})
            for k, name in enumerate(ctx.get("target_names", [])):
                if k < len(e):
                    per[name] = max(per.get(name, 0.0), float(e[k]))
        except Exception:  # noqa: BLE001
            pass

    # 2) 再设当前帧目标
    row = episode.joint_pos[i][source_idx]
    if np.all(np.isfinite(row)):
        values = row.astype(np.float32).reshape(1, -1)
        _apply_joint_target(ctx, values, i)
        ctx["prev_values"] = values.reshape(-1).copy()
    else:
        ctx["prev_values"] = None

    for spec in ctx.get("grip_specs", []):
        closed = bool(spec["closed"][min(i, len(spec["closed"]) - 1)])
        tgt = spec["close_target"] if closed else spec["open_target"]
        art.set_dof_position_targets(
            np.array([[tgt]], dtype=np.float32),
            dof_indices=np.array([spec["dof"]], dtype=np.int64),
        )


def _step_ee(ctx: dict, i: int) -> None:
    episode = ctx["episode"]
    mk = ctx["mk_pose"]
    ctx["pub_l"].publish(mk(episode.left_pos[i], episode.left_quat[i]))
    ctx["pub_r"].publish(mk(episode.right_pos[i], episode.right_quat[i]))
    try:
        ctx["rclpy"].spin_once(ctx["node"], timeout_sec=0.0)
    except Exception:  # noqa: BLE001
        pass


def _finish() -> None:
    """收尾：把关节送回回放前姿态、等稳定、恢复运控、退订、打印统计。

    **顺序很重要**（用户实测验证过）：
      1. 先把关节恢复到回放前的位置（用 teleport）
      2. 等物理几帧稳定
      3. 再激活控制器

    如果直接恢复控制器，OCS2 的初始状态和当前姿态有偏差 → 关节被猛拉，
    表现为"回放结束时突然抽动、异常转身"。这个坑之前修运控解决过。
    """
    import time

    ctx = _STATE.get("ctx") or {}
    _STATE["playing"] = False
    _STATE["cancel"] = False
    _uninstall_update_hook()

    # 1) 送回原位（在恢复运控之前）
    _restore_initial_pose(ctx)

    # 2) 等物理稳定。已退出 update 回调链，这里 sleep 是安全的
    #   （不会重入主循环）。Isaac 自己的循环会推进物理。
    if ctx.get("paused_controllers"):
        time.sleep(0.3)
        _log("姿态已复位，等待物理稳定后交还运控")
        # 3) 恢复运控
        _resume_controllers(ctx)

    if ctx.get("node") is not None:
        try:
            ctx["node"].destroy_node()
        except Exception:  # noqa: BLE001
            pass
    if ctx.get("cm_node") is not None:
        try:
            ctx["cm_node"].destroy_node()
        except Exception:  # noqa: BLE001
            pass

    # 把最终统计留在 _STATE["last_result"]，让 CLI 的 live-status/--wait
    # 也能看到（用户通常只看 CLI 终端，看不到 Isaac 的 stdout）
    if ctx:
        n = ctx.get("frames_done", 0)
        if ctx.get("drive") == "joints":
            me = ctx.get("max_err", 0.0) * 1000
            errs = ctx.get("errs") or []
            p95 = float(np.percentile(errs, 95)) * 1000 if errs else 0.0
            _log(f"回放结束: {n} 帧，关节 max_err={me:.2f} mrad p95={p95:.2f} mrad")
            # 逐关节误差（定位是哪个跟不上，比如腰部）
            per = ctx.get("err_per_joint") or {}
            if per:
                worst = sorted(per.items(), key=lambda kv: -kv[1])[:6]
                _log("  误差最大的关节: " + "  ".join(
                    f"{k}={v*1000:.1f}mrad" for k, v in worst
                ))
            _STATE["last_result"] = {
                "frames": n, "drive": "joints",
                "max_err_mrad": round(me, 2), "p95_err_mrad": round(p95, 2),
                "driven_joints": int(len(ctx.get("target_idx", []))),
                "app_dt_ms": round(float(ctx.get("app_dt", 0.0)) * 1000, 2),
                "joint_api": ctx.get("joint_api", "target"),
                "err_per_joint_mrad": {
                    k: round(v * 1000, 1) for k, v in
                    sorted((per or {}).items(), key=lambda kv: -kv[1])[:10]
                },
            }
        else:
            _log(f"回放结束: 发了 {n} 帧末端位姿")
            _STATE["last_result"] = {"frames": n, "drive": "ee"}

    _STATE.update(episode=None, ctx=None, i=0)
