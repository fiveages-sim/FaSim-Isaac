"""隔离实验：直接给运控发阶跃末端目标，判断「运控到底在不在跟随」。

## 为什么需要
闭环推理里末端 800 帧只动了 5.5mm，而目标动了 8.5mm，两者**方向相反**（cos=-0.96），
末端与目标之间**恒定差 9mm 不收敛**。这有三种完全不同的病因：

  (a) 策略输出太小 / 观测冻结        → 换大阶跃应该能动
  (b) 运控「收到即丢弃」（闸门关闭）  → 连 50mm 阶跃都不动，且 current_target 不回读
  (c) 运控接受了但**跟不动**（下垂）  → 50mm 阶跃只走一小段，方向也可能错

本脚本绕开策略，直接发阶跃目标，并**同时**记录三路信号以区分 (b) 和 (c)：

  /<side>_current_pose          实测末端位姿（持续发布）
  /<side>_current_target        运控内部目标 ← **只有它收到并接受了指令才会动**
  /ocs2_wbc_controller/current_state   手臂使能 / 底盘锁 / 双臂耦合状态
  /fsm_state                    1=HOME 2=HOLD 3=OCS2 4=MOVEJ

## 运控侧的三道闸门（已查源码，任何一道关闭都会「收到即丢弃」）
  1) `PoseBasedReferenceManager::accepting_targets_`
     仅在 FSM==OCS2 时为 true（Ocs2ArmController.cpp:560 syncPoseTargetAcceptance）
  2) `setArmExternalCommandPredicate` → `block_<side>_external_ee_`
     该臂处于 disable wait / disabled / enable wait 时，**该臂**的所有 EE 指令被丢弃
     （WbcCtrlComponent.h:224）
  3) FSM 状态码：HOME/HOLD/MOVEJ 都不接受末端绝对目标

## 为什么要测多个轴
默认同时测 x（前伸，不受重力影响）和 z（上抬，逆重力）。
- x 能动、z 不能动  ⇒ 运控链路 OK，是**逆重力能力/下垂**问题
- x、z 都完全不动    ⇒ 闸门关闭或目标被忽略

## 用法（Isaac 在跑、运控正常）
    cd ~/lerobot_ros2 && source .venv-train/bin/activate
    # 先只看状态，不发任何东西
    python ~/FaSim-Isaac/exts/fasim.replicator.sdg/scripts/probe_ee_tracking.py --check-only
    # 双手 × x/z 全测
    python ~/FaSim-Isaac/exts/fasim.replicator.sdg/scripts/probe_ee_tracking.py
    # 若报「手臂未使能」，再加上 --enable 让它先发 BASE_LOCK + 双臂使能
    python ~/FaSim-Isaac/exts/fasim.replicator.sdg/scripts/probe_ee_tracking.py --enable
"""

from __future__ import annotations

import argparse
import json
import time
from pathlib import Path

import numpy as np

OUT = Path("/tmp/sdg_ee_probe")

FSM_NAMES = {0: "INVALID", 1: "HOME", 2: "HOLD", 3: "OCS2", 4: "MOVEJ"}
FSM_OCS2 = 3
FSM_HOLD = 2
ARM_ENABLED = 1
BASE_LOCKED = 0

CUR_TOPIC = "/{side}_current_pose"
TGT_ECHO_TOPIC = "/{side}_current_target"
CMD_TOPIC = "/{side}_target"
FSM_TOPIC = "/fsm_state"
WBC_TOPIC = "/ocs2_wbc_controller/current_state"
MODE_TOPIC = "/mode_command"


def main() -> int:
    ap = argparse.ArgumentParser(
        description="给运控发阶跃末端目标，验证它是否真的跟随（双手可测）")
    ap.add_argument("--step-mm", type=float, default=50.0, help="阶跃距离（mm）")
    ap.add_argument("--axis", default="x,z",
                    help="测试轴，逗号分隔（默认 x,z：x 不受重力、z 逆重力）")
    ap.add_argument("--side", default="both", choices=["left", "right", "both"],
                    help="测哪只手（默认双手都测）")
    ap.add_argument("--baseline-sec", type=float, default=1.2, help="发指令前的静止观测时长")
    ap.add_argument("--hold-sec", type=float, default=4.0, help="阶跃目标保持时长")
    ap.add_argument("--hz", type=float, default=20.0, help="发布频率")
    ap.add_argument("--enable", action="store_true",
                    help="先发 BASE_LOCK + 双臂使能（仅在确认闸门关闭时使用）")
    ap.add_argument("--switch-ocs2", action="store_true",
                    help="若 FSM 不在 OCS2，自动经 HOLD 切过去")
    ap.add_argument("--check-only", action="store_true",
                    help="只打印话题/闸门状态，不发任何指令")
    ap.add_argument("--no-restore", action="store_true", help="测完不回到起点")
    ap.add_argument("--dry-run", action="store_true", help="只打印将要发的目标")
    ap.add_argument("--ramp", type=int, default=0,
                    help="小增量累积模式：连发 N 次 step-mm（每次以当前位姿为基准），"
                         "看小步长能否累积成大位移。默认 0 = 关闭。")
    ap.add_argument("--sweep", action="store_true",
                    help="扫描模式：对一串步长各做一次短 ramp，找出「前进/后退」的分界。"
                         "用来验证「增量 < 稳态残差 ⇒ 目标反向倒退」这一假设。")
    ap.add_argument("--ramp-per", type=float, default=0.0,
                    help="ramp 模式下每次增量给出的时间（秒）。0 = 用 hold-sec/N。"
                         "调大它可判断「累积慢」是死区还是运控带宽不够。")
    ap.add_argument("--sweep-steps", default="0.5,1,2,3,5,10",
                    help="--sweep 时扫描的步长（mm，逗号分隔）")
    ap.add_argument("--sweep-n", type=int, default=40, help="--sweep 时每个步长发多少帧")
    ap.add_argument("--replay-dataset", metavar="PATH", default="",
                    help="回放数据集的 ee_absolute 绝对轨迹（20Hz）并测量跟随效果。"
                         "这是区分「运控滞后」和「策略输出退化」的决定性实验。")
    ap.add_argument("--episode", type=int, default=0, help="--replay-dataset 用第几个 episode")
    ap.add_argument("--frames", type=int, default=400, help="--replay-dataset 回放多少帧")
    args = ap.parse_args()

    OUT.mkdir(parents=True, exist_ok=True)
    axes = [a.strip() for a in args.axis.split(",") if a.strip() in ("x", "y", "z")]
    if not axes:
        print("ERROR --axis 必须是 x/y/z 的逗号分隔组合")
        return 2
    sides = ["left", "right"] if args.side == "both" else [args.side]

    import rclpy
    from geometry_msgs.msg import Pose
    from rosidl_runtime_py.utilities import get_message
    from rclpy.node import Node

    rclpy.init()
    node = Node("sdg_ee_probe")
    types = dict(node.get_topic_names_and_types())

    def mtype(topic: str):
        """话题存在则返回消息类，否则 None。"""
        if topic not in types or not types[topic]:
            return None
        # 同名话题有多种类型时挑一个能解析的
        for t in types[topic]:
            try:
                return get_message(t)
            except Exception:  # noqa: BLE001
                continue
        return None

    def qos_for(topic: str, durable_hint: str = ""):
        """fsm_state / current_state 是 transient_local 发布的。

        用默认 volatile QoS 订阅它们**收不到任何消息**（踩过：报
        「运控没在跑」，实际是 QoS 不兼容）。这里按话题名匹配 durable。
        """
        from rclpy.qos import (DurabilityPolicy, HistoryPolicy, QoSProfile,
                               ReliabilityPolicy)
        transient = topic in (FSM_TOPIC, WBC_TOPIC) or "state" in durable_hint
        return QoSProfile(
            depth=10,
            history=HistoryPolicy.KEEP_LAST,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=(DurabilityPolicy.TRANSIENT_LOCAL if transient
                        else DurabilityPolicy.VOLATILE),
        )

    # ---------------- 共享状态 ----------------
    cur: dict[str, dict] = {s: {} for s in sides}     # 实测末端位姿
    echo: dict[str, dict] = {s: {} for s in sides}    # 运控内部目标（仅在有指令时发布）
    echo_count = {s: 0 for s in sides}
    fsm_box: list[int] = []
    wbc_box: list = []

    def mk_cur_cb(side):
        def cb(m):
            p = getattr(m, "pose", m)
            d = cur[side]
            d["x"] = float(p.position.x)
            d["y"] = float(p.position.y)
            d["z"] = float(p.position.z)
            d["q"] = (float(p.orientation.x), float(p.orientation.y),
                      float(p.orientation.z), float(p.orientation.w))
            d["t"] = time.monotonic()
        return cb

    def mk_echo_cb(side):
        def cb(m):
            p = getattr(m, "pose", m)
            d = echo[side]
            d["x"] = float(p.position.x)
            d["y"] = float(p.position.y)
            d["z"] = float(p.position.z)
            d["t"] = time.monotonic()
            echo_count[side] += 1
        return cb

    subs = []
    missing = []
    for s in sides:
        t = CUR_TOPIC.format(side=s)
        cls = mtype(t)
        if cls is not None:
            subs.append(node.create_subscription(cls, t, mk_cur_cb(s), 20))
        else:
            missing.append(t)
        t2 = TGT_ECHO_TOPIC.format(side=s)
        cls2 = mtype(t2)
        if cls2 is not None:
            subs.append(node.create_subscription(cls2, t2, mk_echo_cb(s), 20))
    def on_fsm(m) -> None:
        fsm_box.clear()
        fsm_box.append(int(m.data))

    def on_wbc(m) -> None:
        wbc_box.clear()
        wbc_box.append(m)

    if mtype(FSM_TOPIC) is not None:
        subs.append(node.create_subscription(
            mtype(FSM_TOPIC), FSM_TOPIC, on_fsm, qos_for(FSM_TOPIC)))
    if mtype(WBC_TOPIC) is not None:
        subs.append(node.create_subscription(
            mtype(WBC_TOPIC), WBC_TOPIC, on_wbc, qos_for(WBC_TOPIC)))

    def spin_for(sec: float) -> None:
        t_end = time.monotonic() + sec
        while time.monotonic() < t_end:
            rclpy.spin_once(node, timeout_sec=0.02)

    # 等第一帧实测位姿（持续发布，正常 0.2s 内到）
    spin_for(2.0)

    ready = [s for s in sides if "z" in cur[s]]
    stale = [s for s in ready if time.monotonic() - cur[s].get("t", 0) > 1.5]

    # ---------------- A. 环境 / 闸门状态 ----------------
    print("=" * 74)
    print("A. 环境与闸门状态")
    print("=" * 74)

    fsm = fsm_box[0] if fsm_box else None
    fsm_txt = FSM_NAMES.get(fsm, str(fsm))
    gate1 = (fsm == FSM_OCS2)
    print(f"  /fsm_state                 : {fsm} ({fsm_txt})")
    print(f"  闸门1 accepting_targets    : {'✅ 开（FSM=OCS2）' if gate1 else '❌ 关（只有 OCS2 接受末端绝对目标）'}")

    wbc = wbc_box[0] if wbc_box else None
    gates2 = {}
    if wbc is not None:
        print(f"  current_state.base_state   : {wbc.base_state} "
              f"({'LOCKED' if wbc.base_state == BASE_LOCKED else 'UNLOCKED ← 底盘会自己挪'})")
        print(f"  current_state.bimanual     : {wbc.bimanual_state} "
              f"({'COUPLED ← 右臂会被左臂带动' if wbc.bimanual_state == 1 else 'INDEPENDENT'})")
        for s in ("left", "right"):
            st = getattr(wbc, f"{s}_arm_state", None)
            ok = (st == ARM_ENABLED)
            gates2[s] = ok
            print(f"  current_state.{s}_arm_state : {st} "
                  f"({'ENABLED' if ok else 'DISABLED ← 该臂指令会被丢弃'})")
        print(f"  head_state={getattr(wbc, 'head_state', '?')}  "
              f"home_joint_ref={getattr(wbc, 'home_joint_reference_enabled', '?')}")
    else:
        print("  current_state              : ❌ 读不到 /ocs2_wbc_controller/current_state")
        print("     ⇒ 运控没在跑，或话题名不同；闸门2 无法判定")

    print()
    for s in sides:
        t_cur = CUR_TOPIC.format(side=s)
        t_echo = TGT_ECHO_TOPIC.format(side=s)
        t_cmd = CMD_TOPIC.format(side=s)
        print(f"  [{s}] 实测 {t_cur:24s} {'✅' if t_cur in types else '❌ 缺'}   "
              f"内部目标 {t_echo:24s} {'有' if t_echo in types else '缺（空闲时常无发布）'}   "
              f"指令 {t_cmd} {'✅' if t_cmd in types else '❌ 缺'}")

    if missing:
        print(f"\n  ⚠️ 缺实测位姿话题: {missing}")
    if stale:
        print(f"\n  ⚠️ 以下侧的实测位姿已过期（>1.5s 没更新）: {stale}"
              "  ⇒ OCS2 可视化没在跑，或仿真暂停")
    if not ready:
        print("\n  ❌ 一侧实测位姿都没读到。检查：")
        print("     1) Isaac 里 OCS2 控制器是否 active（/ocs2_wbc_controller）")
        print("     2) 是否 source 了与 Isaac 相同的 ROS_DOMAIN_ID / zenoh 配置")
        print("     3) 手动验证：ros2 topic echo /left_current_pose --once")
        node.destroy_node()
        rclpy.shutdown()
        return 1

    print()
    for s in ready:
        c = cur[s]
        print(f"  [{s}] 起点 x={c['x']:+.4f} y={c['y']:+.4f} z={c['z']:+.4f}")

    if args.check_only:
        print("\n[check-only] 未发送任何指令。")
        (OUT / "state.json").write_text(json.dumps({
            "fsm": fsm, "fsm_name": fsm_txt, "gate1_accepting": gate1,
            "gate2_arm_enabled": gates2,
            "base_state": getattr(wbc, "base_state", None) if wbc else None,
            "bimanual_state": getattr(wbc, "bimanual_state", None) if wbc else None,
            "ready_sides": ready, "missing_topics": missing,
        }, indent=2, ensure_ascii=False), encoding="utf-8")
        print(f"状态报告: {OUT}/state.json")
        node.destroy_node()
        rclpy.shutdown()
        return 0

    # ---------------- B. 可选：开闸门 ----------------
    def send_mode(cmd: str) -> None:
        cls = mtype(MODE_TOPIC)
        if cls is None:
            print(f"  [warn] 没有 {MODE_TOPIC} 话题")
            return
        pub = node.create_publisher(cls, MODE_TOPIC, 10)
        m = cls()
        m.data = cmd
        pub.publish(m)
        spin_for(0.35)
        node.destroy_publisher(pub)
        print(f"  → {MODE_TOPIC}: {cmd}")

    def send_fsm(code: int) -> None:
        t = "/fsm_command"
        cls = mtype(t)
        if cls is None:
            print(f"  [warn] 没有 {t} 话题")
            return
        pub = node.create_publisher(cls, t, 10)
        m = cls()
        m.data = int(code)
        pub.publish(m)
        spin_for(0.1)
        node.destroy_publisher(pub)
        print(f"  → {t}: {code} ({FSM_NAMES.get(code, '?')})")

    if args.switch_ocs2 and not gate1:
        print("\n" + "=" * 74)
        print("B0. FSM 切到 OCS2（经 HOLD）")
        print("=" * 74)
        if fsm is not None and fsm != FSM_HOLD:
            send_fsm(FSM_HOLD)
            spin_for(1.0)
        send_fsm(FSM_OCS2)
        spin_for(1.2)
        fsm = fsm_box[0] if fsm_box else None
        gate1 = (fsm == FSM_OCS2)
        print(f"  现在 FSM={fsm} ({FSM_NAMES.get(fsm, '?')})  "
              f"闸门1 {'✅ 开' if gate1 else '❌ 仍关'}")

    if args.enable:
        print("\n" + "=" * 74)
        print("B. 开闸门：BASE_LOCK + 双臂使能")
        print("=" * 74)
        send_mode("BASE_LOCK")
        for s in ready:
            send_mode("LEFT_ARM_ENABLE" if s == "left" else "RIGHT_ARM_ENABLE")
        spin_for(1.2)

    # ---------------- C. 阶跃测试 ----------------
    def make_cmd(cls, p: Pose):
        """构造指令消息：/side_target 通常是纯 Pose，但也兼容 PoseStamped。"""
        if not hasattr(cls, "header"):
            return p
        try:
            msg = cls()
        except Exception:  # noqa: BLE001
            return p
        msg.header.stamp = node.get_clock().now().to_msg()
        msg.header.frame_id = "world"
        msg.pose = p
        return msg

    def cmd_pub_for(side: str):
        t = CMD_TOPIC.format(side=side)
        cls = mtype(t)
        if cls is None:
            return None, None
        return node.create_publisher(cls, t, 10), cls

    def measure(side: str, axis: str, step_m: float) -> dict:
        """静止基线 → 发阶跃并保持 → 记录末端/内部目标的变化。"""
        ai = "xyz".index(axis)
        base = dict(cur[side])                      # 起点（含朝向，朝向保持不变）
        echo_before = dict(echo[side]) if echo[side] else None
        cnt_before = echo_count[side]

        target = Pose()
        target.position.x = base["x"] + (step_m if axis == "x" else 0.0)
        target.position.y = base["y"] + (step_m if axis == "y" else 0.0)
        target.position.z = base["z"] + (step_m if axis == "z" else 0.0)
        (target.orientation.x, target.orientation.y,
         target.orientation.z, target.orientation.w) = base["q"]

        if args.dry_run:
            print(f"  [{side}/{axis}] dry-run 目标 "
                  f"({target.position.x:.4f}, {target.position.y:.4f}, {target.position.z:.4f})")
            return {"side": side, "axis": axis, "dry_run": True}

        # ---- 静止基线（确认末端本身不飘）----
        base_traj = []
        t_end = time.monotonic() + args.baseline_sec
        while time.monotonic() < t_end:
            rclpy.spin_once(node, timeout_sec=0.02)
            base_traj.append((cur[side]["x"], cur[side]["y"], cur[side]["z"]))
        ba = np.array(base_traj, dtype=float)
        baseline_drift = float(np.linalg.norm(ba[-1] - ba[0])) * 1000 if len(ba) > 1 else float("nan")

        # ---- 发阶跃并保持 ----
        pub, cls_cmd = cmd_pub_for(side)
        if pub is None:
            print(f"  ❌ [{side}/{axis}] 缺话题 {CMD_TOPIC.format(side=side)}，跳过")
            return {"side": side, "axis": axis, "error": "no_command_topic"}
        msg = make_cmd(cls_cmd, target)
        traj = []
        tgt_traj = []
        period = 1.0 / max(args.hz, 1.0)
        t_end = time.monotonic() + args.hold_sec
        while time.monotonic() < t_end:
            if hasattr(msg, "header"):
                msg.header.stamp = node.get_clock().now().to_msg()
            pub.publish(msg)
            rclpy.spin_once(node, timeout_sec=0.0)
            traj.append((cur[side]["x"], cur[side]["y"], cur[side]["z"]))
            if echo[side]:
                tgt_traj.append(tuple(echo[side][k] for k in "xyz"))
            time.sleep(period)
        node.destroy_publisher(pub)

        a = np.array(traj, dtype=float)
        cur_disp = float(a[-1, ai] - a[0, ai]) * 1000 if len(a) else float("nan")
        cur_total = float(np.linalg.norm(a[-1] - a[0])) * 1000 if len(a) else float("nan")

        # 内部目标位移 = 运控是否真的接受了这条指令
        tgt_disp = float("nan")
        accepted = None
        if tgt_traj and echo_before is not None:
            ta = np.array(tgt_traj, dtype=float)
            tgt_disp = float(ta[-1, ai] - echo_before[axis]) * 1000
            accepted = abs(tgt_disp) > abs(step_m * 1000) * 0.2
        elif echo_count[side] > cnt_before:
            accepted = True       # 有新消息但拿不到基线，至少说明有响应
        elif echo[side]:
            accepted = False

        residual = float(np.linalg.norm(
            np.array([target.position.x, target.position.y, target.position.z]) - a[-1])) * 1000 \
            if len(a) else float("nan")
        # 残差按轴拆开：能区分"轴向没跟上"和"运动时下垂到别的轴"
        resid_xyz = (np.array([target.position.x, target.position.y, target.position.z]) - a[-1]) * 1000 \
            if len(a) else np.array([np.nan] * 3)
        # 运动过程中末端在非测试轴上的漂移（x 方向运动时的 z 下垂）
        drift_xyz = (a[-1] - a[0]) * 1000 if len(a) else np.array([np.nan] * 3)

        r = {
            "side": side, "axis": axis,
            "step_mm": round(step_m * 1000, 1),
            "baseline_drift_mm": round(baseline_drift, 2),
            "cur_moved_mm": round(cur_disp, 2),
            "cur_total_mm": round(cur_total, 2),
            "tgt_echo_moved_mm": round(tgt_disp, 2) if tgt_disp == tgt_disp else None,
            "accepted": accepted,
            "residual_mm": round(residual, 2),
            "residual_xyz_mm": [round(float(v), 2) for v in resid_xyz],
            "disp_xyz_mm": [round(float(v), 2) for v in drift_xyz],
            # step=0（零阶跃）时没有"跟随比"，这里给 None 而不是除零
            "follow_ratio": (round(cur_disp / (step_m * 1000), 3)
                             if step_m else None),
        }
        ratio_txt = (f"{r['follow_ratio']*100:+.0f}%" if r["follow_ratio"] is not None
                     else "n/a")
        ok = (accepted and r["follow_ratio"] is not None
              and abs(cur_disp) > abs(step_m * 1000) * 0.5)
        if step_m == 0:
            ok = residual < 2.0     # 零阶跃：残差就是稳态误差底
        flag = "✅" if ok else "❌"
        print(f"  {flag} [{side}/{axis}] 阶跃 {step_m*1000:+.1f}mm → 末端 {cur_disp:+.2f}mm "
              f"(跟随 {ratio_txt})  内部目标 {r['tgt_echo_moved_mm']}mm "
              f"(接受={accepted})  残差 {r['residual_mm']}mm  基线漂移 {r['baseline_drift_mm']}mm")
        print(f"       位移 xyz={r['disp_xyz_mm']}  残差 xyz={r['residual_xyz_mm']}")
        return r

    # ---------------- C2. 小增量累积 ----------------
    def ramp(side: str, axis: str, step_m: float, n: int, verbose: bool = True) -> dict:
        """连发 N 次小增量，每次以**当前实测位姿**为基准前进一步。

        这是策略真实工作方式（增量式指令）的等价物。
        与单次大阶跃的区别：小目标可能被死区吃掉，但如果每次都基于当前位姿
        重新叠加，就能逐步爬出去 —— 那说明"策略输出小"不是死锁，只是慢。
        """
        ai = "xyz".index(axis)
        pub, cls_cmd = cmd_pub_for(side)
        if pub is None:
            print(f"  ❌ [{side}] 缺话题，跳过 ramp")
            return {"side": side, "axis": axis, "error": "no_topic"}
        start = np.array([cur[side][k] for k in "xyz"], dtype=float)
        hist = []
        per = args.ramp_per if args.ramp_per > 0 else max(args.hold_sec / max(n, 1), 0.12)
        for i in range(n):
            base = dict(cur[side])
            p = Pose()
            p.position.x = base["x"] + (step_m if axis == "x" else 0.0)
            p.position.y = base["y"] + (step_m if axis == "y" else 0.0)
            p.position.z = base["z"] + (step_m if axis == "z" else 0.0)
            (p.orientation.x, p.orientation.y,
             p.orientation.z, p.orientation.w) = base["q"]
            msg = make_cmd(cls_cmd, p)
            if not args.dry_run:
                t_end = time.monotonic() + per
                while time.monotonic() < t_end:
                    if hasattr(msg, "header"):
                        msg.header.stamp = node.get_clock().now().to_msg()
                    pub.publish(msg)
                    rclpy.spin_once(node, timeout_sec=0.0)
                    time.sleep(1.0 / max(args.hz, 1.0))
            now = np.array([cur[side][k] for k in "xyz"], dtype=float)
            hist.append(round(float(now[ai] - start[ai]) * 1000, 2))
        node.destroy_publisher(pub)
        end = np.array([cur[side][k] for k in "xyz"], dtype=float)
        total_axis = float(end[ai] - start[ai]) * 1000
        total = float(np.linalg.norm(end - start)) * 1000
        ideal = step_m * 1000 * n
        r = {
            "side": side, "axis": axis, "mode": "ramp", "n": n,
            "step_mm": round(step_m * 1000, 2), "ideal_mm": round(ideal, 1),
            "total_axis_mm": round(total_axis, 2),
            "total_mm": round(total, 2),
            "cum_ratio": round(total_axis / ideal, 3) if ideal else None,
            "progress_mm": hist,
        }
        print(f"  [{side}/{axis}] {n}×{step_m*1000:.1f}mm 累积：目标 {ideal:.0f}mm "
              f"→ 实际 {total_axis:+.2f}mm (累积率 {(r['cum_ratio'] or 0)*100:+.0f}%)")
        if verbose:
            print(f"       逐步累积 {hist}")
        return r

    # ---------------- F. 回放数据集绝对轨迹 ----------------
    def replay_dataset(path: str, side: str, ep: int, n: int) -> dict:
        """按 20Hz 回放真实采集的 ee_absolute 轨迹，测末端跟随。

        关键区别：这是**绝对位姿**序列（不是「当前位姿 + 增量」），
        与推理时 n_action_steps=1 的工况完全一致。
        """
        try:
            import pandas as pd
        except Exception as exc:  # noqa: BLE001
            print(f"  ❌ 需要 pandas: {exc}")
            return {}
        import glob
        fs = sorted(glob.glob(f"{path}/data/**/*.parquet", recursive=True))
        if not fs:
            print(f"  ❌ {path} 下没找到 parquet")
            return {}
        df = pd.concat([pd.read_parquet(f) for f in fs], ignore_index=True)
        acts = np.stack(df["action"].values)
        eps = df["episode_index"].values
        uniq = sorted(set(eps.tolist()))
        if ep >= len(uniq):
            print(f"  ❌ episode {ep} 不存在（共 {len(uniq)} 个）")
            return {}
        m = eps == uniq[ep]
        a = acts[m]
        print(f"  用 episode {uniq[ep]}，共 {len(a)} 帧")
        off = 0 if side == "left" else 7
        pub, cls_cmd = cmd_pub_for(side)
        if pub is None:
            return {}
        start = np.array([cur[side][k] for k in "xyz"], dtype=float)
        # 对齐：把轨迹整体平移，使第 0 帧等于当前位姿（避免大跳变）
        shift = start - a[0, off:off + 3]
        traj, tgts = [], []
        n = min(n, len(a))
        period = 1.0 / args.hz
        for i in range(n):
            p = Pose()
            pos = a[i, off:off + 3] + shift
            p.position.x, p.position.y, p.position.z = (float(x) for x in pos)
            q = a[i, off + 3:off + 7]           # xyzw（导出时已转好）
            p.orientation.x, p.orientation.y = float(q[0]), float(q[1])
            p.orientation.z, p.orientation.w = float(q[2]), float(q[3])
            msg = make_cmd(cls_cmd, p)
            pub.publish(msg)
            rclpy.spin_once(node, timeout_sec=0.0)
            traj.append((cur[side]["x"], cur[side]["y"], cur[side]["z"]))
            tgts.append(tuple(float(x) for x in pos))
            time.sleep(period)
        node.destroy_publisher(pub)

        c = np.array(traj, dtype=float)
        t = np.array(tgts, dtype=float)
        cur_disp = c[-1] - c[0]
        tgt_disp = t[-1] - t[0]
        # 全程跟随误差（每帧实测 vs 当帧目标）
        err = np.linalg.norm(c - t, axis=1) * 1000
        # 方向一致性（只用够大的位移判断）
        nc, nt = np.linalg.norm(cur_disp), np.linalg.norm(tgt_disp)
        cos = float(cur_disp @ tgt_disp / (nc * nt)) if nc > 1e-9 and nt > 1e-9 else float("nan")
        r = {
            "side": side, "mode": "replay", "frames": n,
            "cur_disp_mm": round(float(nc) * 1000, 2),
            "tgt_disp_mm": round(float(nt) * 1000, 2),
            "follow_ratio": round(float(nc / nt), 3) if nt > 1e-9 else None,
            "cos": round(cos, 3),
            "err_median_mm": round(float(np.median(err)), 2),
            "err_p90_mm": round(float(np.percentile(err, 90)), 2),
            "err_first_mm": round(float(err[0]), 2),
            "err_last_mm": round(float(err[-1]), 2),
        }
        print(f"  [{side}] 回放 {n} 帧：目标位移 {r['tgt_disp_mm']}mm  "
              f"实际 {r['cur_disp_mm']}mm  (跟随 {r['follow_ratio']}×)  cos={r['cos']}")
        print(f"       跟随误差 中位 {r['err_median_mm']}mm  p90 {r['err_p90_mm']}mm  "
              f"首 {r['err_first_mm']}mm 末 {r['err_last_mm']}mm")
        return r

    if args.replay_dataset:
        print()
        print("=" * 74)
        print("F. 回放数据集绝对轨迹（与 n_action_steps=1 工况完全一致）")
        print("=" * 74)
        rep = []
        for s in ready:
            rep.append(replay_dataset(args.replay_dataset, s, args.episode, args.frames))
            spin_for(0.5)
        print()
        print("  " + "-" * 70)
        for r in rep:
            if not r:
                continue
            if r["follow_ratio"] and r["follow_ratio"] > 0.8 and (r["cos"] or 0) > 0.9:
                print(f"  ✅ {r['side']}: 运控能跟上真实轨迹（跟随 {r['follow_ratio']}×, cos={r['cos']}）")
                print("     ⇒ 运控侧完全没问题。闭环不动的原因在**策略输出退化成小增量**。")
            elif (r["cos"] or 0) < 0.5:
                print(f"  ❌ {r['side']}: 方向都不对（cos={r['cos']}）⇒ 运控没有跟随轨迹")
            else:
                print(f"  ⚠️ {r['side']}: 部分跟随（{r['follow_ratio']}×）"
                      f" 稳态误差 {r['err_median_mm']}mm")
        print("  " + "-" * 70)
        (OUT / "replay.json").write_text(
            json.dumps(rep, indent=2, ensure_ascii=False), encoding="utf-8")
        print(f"\n报告: {OUT}/replay.json")
        node.destroy_node()
        rclpy.shutdown()
        return 0

    print()
    print("=" * 74)
    if args.sweep:
        print(f"E. 步长扫描（每档 {args.sweep_n} 帧，{1/args.hz*1000:.0f}ms/帧）")
        print("   假设：稳态残差 ≈2.5mm；每步增量 < 残差 ⇒ "
              "target_new = current + step ≈ target_old - (resid - step) 会**倒退**")
        print("=" * 74)
        sweep_out = []
        for s in ready:
            print(f"\n--- {s} ---")
            for st in [float(x) for x in args.sweep_steps.split(",") if x.strip()]:
                r = ramp(s, "x", st / 1000.0, args.sweep_n, verbose=False)
                r["step_mm"] = st
                sweep_out.append(r)
                arrow = "前进" if r["total_axis_mm"] > 0.3 else ("倒退" if r["total_axis_mm"] < -0.3 else "不动")
                print(f"  步长 {st:5.1f}mm × {args.sweep_n:3d} 帧 → "
                      f"{r['total_axis_mm']:+8.2f}mm  【{arrow}】"
                      f"  每帧均 {r['total_axis_mm']/args.sweep_n:+.3f}mm")
                spin_for(0.3)
        print("\n  " + "-" * 70)
        for s in ready:
            rs = [r for r in sweep_out if r["side"] == s]
            pos = [r["step_mm"] for r in rs if r["total_axis_mm"] > 0.3]
            neg = [r["step_mm"] for r in rs if r["total_axis_mm"] < -0.3]
            if pos and neg:
                lo, hi = max(neg), min(pos)
                print(f"  {s}: 分界在 {lo}~{hi}mm 之间（<{lo} 倒退，>{hi} 前进）")
                print(f"       ⇒ 与稳态残差 ≈2.5mm 的预测"
                      + ("**吻合**" if lo <= 2.5 <= hi + 1 else "不完全吻合，需再查"))
            elif pos:
                print(f"  {s}: 所有步长都前进（最小 {min(pos)}mm）")
            elif neg:
                print(f"  {s}: 所有步长都倒退（最大 {max(neg)}mm）← 目标在跑偏，不只是慢")
        print("  " + "-" * 70)
        (OUT / "sweep.json").write_text(
            json.dumps(sweep_out, indent=2, ensure_ascii=False), encoding="utf-8")
        print(f"\n报告: {OUT}/sweep.json")
        node.destroy_node()
        rclpy.shutdown()
        return 0

    print(f"C. 阶跃测试  轴={axes}  阶跃={args.step_mm:.0f}mm  保持={args.hold_sec:.1f}s  "
          f"频率={args.hz:.0f}Hz")
    print("=" * 74)

    results = []
    ramp_results = []
    for s in ready:
        print(f"\n--- {s} ---")
        if args.ramp > 0:
            for ax in axes:
                ramp_results.append(ramp(s, ax, args.step_mm / 1000.0, args.ramp))
                spin_for(0.3)
        else:
            for ax in axes:
                results.append(measure(s, ax, args.step_mm / 1000.0))
                spin_for(0.4)        # 让上一步的影响衰减干净

        # 回到起点（避免累积偏移影响下一只手的判断）
        if not args.no_restore and not args.dry_run:
            pub, cls_cmd = cmd_pub_for(s)
            if pub is not None:
                p = Pose()
                p.position.x, p.position.y, p.position.z = cur[s]["x"], cur[s]["y"], cur[s]["z"]
                (p.orientation.x, p.orientation.y,
                 p.orientation.z, p.orientation.w) = cur[s]["q"]
                msg = make_cmd(cls_cmd, p)
                t_end = time.monotonic() + 1.5
                while time.monotonic() < t_end:
                    if hasattr(msg, "header"):
                        msg.header.stamp = node.get_clock().now().to_msg()
                    pub.publish(msg)
                    rclpy.spin_once(node, timeout_sec=0.0)
                    time.sleep(1.0 / max(args.hz, 1.0))
                node.destroy_publisher(pub)

    print()
    print("=" * 74)
    print("D. 结论")
    print("=" * 74)

    real = [r for r in results if not r.get("dry_run") and r.get("follow_ratio") is not None]
    zero = [r for r in results if not r.get("dry_run") and r.get("follow_ratio") is None]

    if ramp_results:
        print("  小增量累积（策略的真实工作方式）:")
        bad = []
        for r in ramp_results:
            cr = r.get("cum_ratio")
            if cr is None:
                continue
            mark = "✅" if cr >= 0.7 else "⚠️"
            if cr < 0.7:
                bad.append(f"{r['side']}/{r['axis']}")
            print(f"      {mark} {r['side']}/{r['axis']}: {r['n']}×{r['step_mm']}mm "
                  f"→ {r['total_axis_mm']}mm / 理想 {r['ideal_mm']}mm "
                  f"(累积率 {cr*100:+.0f}%)")
        if bad:
            print(f"\n      ⚠️ 这些组合累积率 <70%: {bad}")
            print("         ⇒ 小增量被死区/下垂吃掉了绝大部分，策略即使输出正确也走不到目标。")
            print("            这解释了 L3 里 800 帧只动 8.5mm 的现象。")
        else:
            print("\n      ✅ 小增量能可靠累积 ⇒ 策略只要持续朝同一方向给增量，末端就会走出去。")
            print("         ⇒ 死锁的真正原因在策略侧（观测不变 → 输出不变），不在运控。")
        print()
    if zero:
        print("  零阶跃（把当前位置原样发回）——测运控稳态残差底:")
        for r in zero:
            print(f"      {r['side']}/{r['axis']}: 残差 {r['residual_mm']}mm "
                  f"(基线漂移 {r['baseline_drift_mm']}mm)")
        rr = [r["residual_mm"] for r in zero if r["residual_mm"] == r["residual_mm"]]
        if rr:
            floor = sum(rr) / len(rr)
            print(f"      → 稳态残差底 ≈ {floor:.1f}mm。闭环里任何 <{floor:.0f}mm 的位移"
                  f"都淹没在这个底里，不能当'末端跟随'的证据。")
        print()

    if not real:
        print("  没有有效测试结果。")
    else:
        any_accepted = any(r["accepted"] for r in real)
        any_followed = any(abs(r["follow_ratio"]) >= 0.5 for r in real)
        step_abs = abs(args.step_mm)

        if not any_accepted:
            print("  ❌ 运控**没有接受**任何一条指令（内部目标不动）")
            print("     ⇒ 闸门是关的。按顺序查：")
            print("        1) FSM 是否 = OCS2（现在 " + str(fsm) + "/" + str(fsm_txt) + "）"
                  + ("" if gate1 else "  ← 用 --switch-ocs2"))
            if gates2:
                closed = [s for s, ok in gates2.items() if not ok]
                if closed:
                    print(f"        2) 这些臂未使能: {closed}  ← 用 --enable")
            print("        3) 该臂是否卡在 enable hold（当前状态看不到，需看运控日志）")
        elif any_followed:
            good = [f"{r['side']}/{r['axis']}" for r in real if abs(r["follow_ratio"]) >= 0.5]
            print(f"  ✅ 运控确实能跟随（{', '.join(good)}）")
            print("     ⇒ 运控链路没问题。机器人不动的原因是**策略输出太小/观测冻结**：")
            print("        - chunk_size=100 配 n_action_steps=1，只执行第 1 帧就重预测")
            print("        - 观测几乎不变 → 预测几乎不变 → 死锁")
            print("     ⇒ 下一步：n_action_steps 提到 ~50 重训，或闭环里做 action chunk 队列")
        else:
            # 接受了但跟不动 —— 逐轴分析
            print("  ⚠️ 运控**接受了指令但跟不上**（内部目标变了，末端没跟上）")
            by_axis: dict[str, list] = {}
            for r in real:
                by_axis.setdefault(r["axis"], []).append(r)
            for ax, rs in by_axis.items():
                ratios = [r["follow_ratio"] for r in rs]
                avg = sum(ratios) / len(ratios)
                note = ""
                if ax == "z":
                    note = "  ← 逆重力轴，跟不动 = 下垂/增益不足/负载补偿差"
                elif ax in ("x", "y"):
                    note = "  ← 非重力轴"
                print(f"      轴 {ax}: 平均跟随 {avg*100:+.0f}%  "
                      f"各侧 {[r['side'] + ':' + str(round(r['follow_ratio']*100)) + '%' for r in rs]}"
                      + note)
            xs = by_axis.get("x", [])
            zs = by_axis.get("z", [])
            if xs and zs:
                x_ok = all(abs(r["follow_ratio"]) > 0.5 for r in xs)
                z_ok = all(abs(r["follow_ratio"]) > 0.5 for r in zs)
                if x_ok and not z_ok:
                    print("\n      ⇒ x 能动、z 不能动：**逆重力/下垂问题**，不是死锁。")
                    print("        查：运控重力补偿参数、手臂 enable hold 是否真解除、")
                    print("            BASE_LOCK 是否生效（底盘在偷偷挪会吃掉目标位移）")
                elif not x_ok and not z_ok:
                    print("\n      ⇒ 所有轴都跟不动：运控接收正常但**控制器没在跟踪**。")
                    print("        查：OCS2 solver 是否在收敛（看运控终端有无 QP infeasible / MPC 失败）")
                    print("            movel_duration 是否过长（目标被当成慢速 moveL）")
                    print(f"            残差是否恒定（现在 {[r['residual_mm'] for r in real]}mm）")
            print(f"\n      阶跃 {step_abs:.0f}mm 时末端只动了 "
                  f"{[r['cur_moved_mm'] for r in real]}mm —— 目标幅度本身可能不够大，")
            print(f"      可试 --step-mm 150 排除「小目标被死区吃掉」。")

    (OUT / "report.json").write_text(
        json.dumps({"args": vars(args), "fsm": fsm, "gate1_accepting": gate1,
                    "gate2_arm_enabled": gates2,
                    "results": results, "ramp_results": ramp_results},
                   indent=2, ensure_ascii=False), encoding="utf-8")
    print(f"\n报告: {OUT}/report.json")

    node.destroy_node()
    rclpy.shutdown()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
