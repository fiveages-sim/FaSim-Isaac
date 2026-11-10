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
    return {
        "playing": _STATE["playing"],
        "paused": _STATE["paused"],
        "frame": _STATE["frame"],
        "total": _STATE["total"],
        "t": float(ep.t[_STATE["frame"]]) if ep is not None and _STATE["frame"] < len(ep) else 0.0,
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
    n_end = n_total if end is None else min(n_total, int(end))
    if n_end <= n_start:
        return {"ok": False, "error": f"空区间 {n_start}..{n_end}"}

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

        ctx.update(art=art, target_idx=target_idx, source_idx=source_idx,
                   grip_specs=grip_specs)
        # 手臂 drive gain —— 与 render_episode 一致。
        # 不设的话用资产默认值（Dex1 的 gain 是给夹爪调的），手臂追不上
        # 目标，实测关节误差 268 mrad。
        try:
            n = len(target_idx)
            k, d = 100000.0, 10000.0
            art.set_dof_gains(
                stiffnesses=np.full((1, n), k, dtype=np.float32),
                dampings=np.full((1, n), d, dtype=np.float32),
                dof_indices=target_idx,
            )
            _log(f"手臂 drive gain K={k:g} D={d:g}（{n} 个关节）")
        except Exception as exc:  # noqa: BLE001
            _log(f"WARN 设手臂 gain 失败: {exc}")
        _log(f"关节驱动 {len(target_idx)} 个，帧 {n_start}..{n_end}")

    timeline = omni.timeline.get_timeline_interface()
    if not timeline.is_playing():
        timeline.play()

    _STATE.update(playing=True, paused=False, frame=n_start, total=n_end,
                  episode=episode, cancel=False, ctx=ctx, i=n_start,
                  emit_acc=0.0)

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

    app_dt = _ensure_app_dt(ctx)
    frame_dt = ctx.get("frame_dt", 0.05)

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
    cb = ctx.get("on_frame")
    if cb is not None:
        try:
            cb(i, {"t": float(episode.t[i]), "frame": i})
        except Exception:  # noqa: BLE001
            pass
    fd = ctx["frames_done"]
    if fd % 50 == 0:
        me = ctx.get("max_err", 0.0) * 1000
        _log(f"  {i}/{n_end}  max_err={me:.1f}mrad  ({fd} 帧)")

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
    """实测 app.update() 一次推进多少仿真秒（只测一次）。"""
    if ctx.get("app_dt"):
        return float(ctx["app_dt"])
    app_dt = 0.016666
    try:
        import omni.kit.app
        import omni.timeline

        tl = omni.timeline.get_timeline_interface()
        if not tl.is_playing():
            tl.play()
        t0 = float(tl.get_current_time())
        omni.kit.app.get_app().update()
        t1 = float(tl.get_current_time())
        if t1 > t0:
            app_dt = t1 - t0
        _log(f"实测每次 update 推进 {app_dt*1000:.2f} ms 仿真时间")
    except Exception as exc:  # noqa: BLE001
        _log(f"WARN 测 app_dt 失败（用 16.67ms）: {exc}")
    ctx["app_dt"] = app_dt
    return app_dt


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
            e = float(np.max(np.abs(act - prev)))
            ctx.setdefault("errs", []).append(e)
            ctx["max_err"] = max(ctx.get("max_err", 0.0), e)
        except Exception:  # noqa: BLE001
            pass

    # 2) 再设当前帧目标
    row = episode.joint_pos[i][source_idx]
    if np.all(np.isfinite(row)):
        values = row.astype(np.float32).reshape(1, -1)
        art.set_dof_position_targets(values, dof_indices=target_idx)
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
    """收尾：退订、销毁 ROS 节点、打印统计。"""
    ctx = _STATE.get("ctx") or {}
    _STATE["playing"] = False
    _STATE["cancel"] = False
    _uninstall_update_hook()

    if ctx.get("node") is not None:
        try:
            ctx["node"].destroy_node()
        except Exception:  # noqa: BLE001
            pass

    if ctx:
        n = ctx.get("frames_done", 0)
        if ctx.get("drive") == "joints":
            me = ctx.get("max_err", 0.0) * 1000
            errs = ctx.get("errs") or []
            p95 = float(np.percentile(errs, 95)) * 1000 if errs else 0.0
            _log(f"回放结束: {n} 帧，关节 max_err={me:.2f} mrad p95={p95:.2f} mrad")
        else:
            _log(f"回放结束: 发了 {n} 帧末端位姿")

    _STATE.update(episode=None, ctx=None, i=0)
