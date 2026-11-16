#!/usr/bin/env python3
"""推理复盘探针：把「看不到的画面」变成数字 + 图。

## 为什么需要
闭环推理失败时，人只能看到"手伸出去又缩回来"，看不到：
  - 策略**原始输出**走的是哪条轨迹（是不是本来就想缩回）
  - 下发的目标 vs 运控实际到达位置差多少（是不是没跟到）
  - 夹爪什么时候闭合、闭合时手离试管多远（是不是抓空）
  - 底盘到底动了多少、什么方向（是不是 base_lock 失效）
  - 三路相机画面到底是什么（是不是分布外）

这个脚本在推理进行时**并行运行**，按固定频率采样并落盘，结束后给出
一份可读报告 + 拼图，不需要看 GUI。

## 用法（推理已经在跑的时候，另开终端）
```bash
cd ~/lerobot_ros2 && source /opt/ros/jazzy/setup.bash
source .venv-train/bin/activate
python ~/FaSim-Isaac/exts/fasim.replicator.sdg/scripts/probe_infer_run.py --seconds 60
```
默认把结果写到 `/tmp/sdg_infer_probe/<时间戳>/`：
  - `samples.csv`    每帧一行（目标/实测/夹爪/底盘/关节力矩）
  - `frames/`        每 N 帧存一次三路相机拼图 PNG
  - `report.txt`     结论文本（含自动判据）

## 判据（自动给结论，不用人看）
| 现象 | 判据 |
|---|---|
| 运控没跟上 | `reach_ratio = 实测位移 / 目标位移` 明显 < 0.8 |
| 底盘真的在漂 | `base_drift_mm` 随时间单调增长，且 `base_state == 0(LOCKED)` |
| base_lock 丢了 | `/ocs2_wbc_controller/current_state.base_state != 0` |
| 抓空 | 夹爪闭合帧上 `hand_to_tube_mm` 远大于试管半径+指厚 |
| 策略自己在缩回 | **目标**轨迹出现"前进后又后退"，且与实测同向（不是没跟到）|
| home_joint 在拽手 | 无外部指令时关节力矩出现指向 HOME 的持续分量 |
"""

from __future__ import annotations

import argparse
import csv
import json
import time
from pathlib import Path

import numpy as np

OUT_ROOT = Path("/tmp/sdg_infer_probe")


def _log(msg: str) -> None:
    print(f"[probe] {msg}", flush=True)


# ------------------------------------------------------------------ ROS 侧


class Collector:
    """订阅推理期间需要的一切话题。"""

    def __init__(self, node, *, tube_prim_hint: str = ""):
        from geometry_msgs.msg import Pose, PoseStamped
        from sensor_msgs.msg import Image, JointState
        import rclpy
        from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy

        self.node = node
        self.rclpy = rclpy
        self.latest: dict = {}
        self.counts: dict = {"left_target": 0, "right_target": 0,
                             "joint_states": 0, "wbc_state": 0, "images": 0}
        self.history: list[dict] = []

        latched = QoSProfile(depth=1)
        latched.durability = DurabilityPolicy.TRANSIENT_LOCAL
        latched.reliability = ReliabilityPolicy.RELIABLE
        be = QoSProfile(depth=10)
        be.reliability = ReliabilityPolicy.BEST_EFFORT

        # ⚠ 类型必须对：运控发布的 *_current_pose / *_current_target 是
        # PoseStamped；只有我们下发的 /<side>_target 是纯 Pose。
        # 订阅类型写错**不会报错**，只是永远收不到消息（踩过）。
        self.node.create_subscription(
            PoseStamped, "/left_current_pose", self._mk("left_cur"), 10)
        self.node.create_subscription(
            PoseStamped, "/right_current_pose", self._mk("right_cur"), 10)
        self.node.create_subscription(
            PoseStamped, "/left_current_target", self._mk("left_tgt_echo"), 10)
        self.node.create_subscription(
            PoseStamped, "/right_current_target", self._mk("right_tgt_echo"), 10)
        # 底盘/躯干位姿：判断"底盘是否在动"的唯一直接证据。
        # body_current_pose 是躯干 frame 在 world 下的位姿；底盘移动时它会跟着走。
        self.node.create_subscription(
            PoseStamped, "/body_current_pose", self._mk("body_cur"), 10)
        # 我们下发的目标（纯 Pose）
        self.node.create_subscription(
            Pose, "/left_target", self._mk("left_cmd", count="left_target"), 10)
        self.node.create_subscription(
            Pose, "/right_target", self._mk("right_cmd", count="right_target"), 10)
        self.node.create_subscription(
            JointState, "/joint_states", self._joint, 10)

        # 夹爪命令：target_command(Int32 0/1) 与 target_percent(Float64)。
        # **必须记** —— 画面里"夹爪全程张开、夹不住试管"这种情况，
        # 只有对比"策略要求的状态"和"实际收到/实际位置"才能定位。
        from std_msgs.msg import Float64, Int32
        for side in ("left", "right"):
            self.node.create_subscription(
                Int32, f"/{side}_gripper_controller/target_command",
                self._mk(f"{side}_grip_cmd", count=f"{side}_grip_target"), 10)
            self.node.create_subscription(
                Float64, f"/{side}_gripper_controller/target_percent",
                self._mk(f"{side}_grip_pct"), 10)

        # WBC 状态：transient_local，否则订不到
        try:
            from rosidl_runtime_py.utilities import get_message
            msg_cls = get_message("arms_ros2_control_msgs/msg/WbcCurrentState")
            for topic in ("/ocs2_wbc_controller/current_state", "/wbc_current_state"):
                try:
                    self.node.create_subscription(
                        msg_cls, topic, self._wbc, latched)
                    _log(f"订阅 {topic}（transient_local）")
                    break
                except Exception:  # noqa: BLE001
                    continue
        except Exception as exc:  # noqa: BLE001
            _log(f"WARN 无法订阅 WBC 状态: {exc}")

        # 相机（best_effort 才收得到 Isaac 的 Image）
        for name, topic in (
            ("head", "/head_camera/rgb"),
            ("wrist_left", "/left_hand_camera/rgb"),
            ("wrist_right", "/right_hand_camera/rgb"),
        ):
            self.node.create_subscription(
                Image, topic, self._mk_img(name), be)

        # FSM
        try:
            from std_msgs.msg import Int32
            self.node.create_subscription(Int32, "/fsm_state", self._mk("fsm"), latched)
        except Exception:  # noqa: BLE001
            pass

        _log("订阅完成")

    def _mk(self, key: str, count: str | None = None):
        def cb(msg):
            self.latest[key] = msg
            if count:
                self.counts[count] = self.counts.get(count, 0) + 1
        return cb

    def _val(self, key: str):
        """从缓存消息里取标量（Int32/Float64）。"""
        m = self.latest.get(key)
        if m is None:
            return None
        return getattr(m, "data", None)

    def _joint(self, msg):
        self.latest["joint_names"] = list(msg.name)
        self.latest["joint_pos"] = np.asarray(msg.position, dtype=np.float64)
        self.latest["joint_eff"] = (np.asarray(msg.effort, dtype=np.float64)
                                    if msg.effort else None)
        self.counts["joint_states"] += 1

    def _wbc(self, msg):
        self.latest["wbc"] = {
            "base_state": int(getattr(msg, "base_state", -1)),
            "left_arm_state": int(getattr(msg, "left_arm_state", -1)),
            "right_arm_state": int(getattr(msg, "right_arm_state", -1)),
            "body_state": int(getattr(msg, "body_state", -1)),
            "home_joint_reference_enabled": bool(
                getattr(msg, "home_joint_reference_enabled", False)),
        }
        self.counts["wbc_state"] += 1

    def _mk_img(self, name: str):
        def cb(msg):
            h, w = msg.height, msg.width
            buf = np.frombuffer(msg.data, dtype=np.uint8)
            try:
                arr = buf.reshape(h, w, -1)[..., :3]
            except Exception:  # noqa: BLE001
                return
            self.latest[f"img_{name}"] = arr
            self.counts["images"] = self.counts.get("images", 0) + 1
        return cb

    @staticmethod
    def _pose(msg) -> dict:
        """取位置/姿态。兼容 Pose 与 PoseStamped（运控发的是 PoseStamped）。"""
        p0 = getattr(msg, "pose", msg)
        p, o = p0.position, p0.orientation
        return {"pos": np.array([p.x, p.y, p.z], dtype=np.float64),
                "quat": np.array([o.x, o.y, o.z, o.w], dtype=np.float64)}

    def snapshot(self, t: float) -> dict:
        row: dict = {"t": round(t, 4)}
        for side in ("left", "right"):
            for src in ("cur", "tgt_echo", "cmd"):
                k = f"{side}_{src}"
                if k in self.latest:
                    d = self._pose(self.latest[k])
                    for i, ax in enumerate("xyz"):
                        row[f"{k}_{ax}"] = round(float(d["pos"][i]), 5)
                    for i, ax in enumerate("xyzw"):
                        row[f"{k}_q{ax}"] = round(float(d["quat"][i]), 5)
        # 底盘/躯干（底盘在动时它必然跟着动）
        if "body_cur" in self.latest:
            d = self._pose(self.latest["body_cur"])
            for i, ax in enumerate("xyz"):
                row[f"body_{ax}"] = round(float(d["pos"][i]), 5)
        if "wbc" in self.latest:
            row.update({f"wbc_{k}": v for k, v in self.latest["wbc"].items()})
        if "fsm" in self.latest:
            try:
                row["fsm"] = int(getattr(self.latest["fsm"], "data", -1))
            except Exception:  # noqa: BLE001
                pass
        jn = self.latest.get("joint_names") or []
        jp = self.latest.get("joint_pos")
        je = self.latest.get("joint_eff")
        for i, n in enumerate(jn):
            if n in ("waist_yaw_joint", "waist_pitch_joint") or "wheel" in n.lower():
                if jp is not None and i < len(jp):
                    row[f"q_{n}"] = round(float(jp[i]), 5)
                if je is not None and i < len(je):
                    row[f"tau_{n}"] = round(float(je[i]), 4)
        # 夹爪：命令（0/1）与关节实际位置
        for side in ("left", "right"):
            v = self._val(f"{side}_grip_cmd")
            if v is not None:
                row[f"grip_cmd_{side}"] = int(v)
            p = self._val(f"{side}_grip_pct")
            if p is not None:
                row[f"grip_pct_{side}"] = round(float(p), 3)
            gname = f"{side}_gripper_joint"
            if gname in jn and jp is not None:
                i = jn.index(gname)
                if i < len(jp):
                    row[f"grip_q_{side}"] = round(float(jp[i]), 5)
        # 底盘：用左右轮速 + waist 关节近似"是否在动"
        for n in ("Left_Wheel_Joint", "Right_Wheel_Joint"):
            if n in jn and jp is not None:
                i = jn.index(n)
                if i < len(jp):
                    row[f"q_{n}"] = round(float(jp[i]), 6)
        row["n_joint_states"] = self.counts.get("joint_states", 0)
        row["n_left_target"] = self.counts.get("left_target", 0)
        row["n_right_target"] = self.counts.get("right_target", 0)
        return row


# ------------------------------------------------------------------ 图像


def montage(images: dict[str, np.ndarray], path: Path, step: int = 0) -> None:
    """三路相机横拼，并在左上角写帧号。"""
    import cv2

    order = ["head", "wrist_left", "wrist_right"]
    tiles = []
    for k in order:
        img = images.get(k)
        if img is None:
            img = np.zeros((480, 640, 3), dtype=np.uint8)
        img = np.asarray(img, dtype=np.uint8)
        # 统一到 640x480（推理侧也是 resize 到训练尺寸）
        if img.shape[:2] != (480, 640):
            img = cv2.resize(img, (640, 480), interpolation=cv2.INTER_AREA)
        img = cv2.copyMakeBorder(img, 24, 0, 0, 0, cv2.BORDER_CONSTANT,
                                 value=(0, 0, 0))
        cv2.putText(img, f"{k}  f{step}", (6, 17),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1, cv2.LINE_AA)
        tiles.append(img)
    out = np.concatenate(tiles, axis=1)
    cv2.imwrite(str(path), out)


# ------------------------------------------------------------------ 报告


def analyse(rows: list[dict], out_dir: Path, img_steps: list[int]) -> str:
    lines: list[str] = []
    add = lines.append

    if not rows:
        return "没有采到任何样本 —— 推理没在跑，或话题名不对。"

    t = np.array([r["t"] for r in rows], dtype=np.float64)
    add(f"采样 {len(rows)} 帧，时长 {t[-1]-t[0]:.1f}s")

    # ---- 话题是否真的有数据 ----
    add("\n[1] 话题计数（推理结束时的累计值）")
    last = rows[-1]
    for k in ("n_joint_states", "n_left_target", "n_right_target"):
        add(f"  {k:18s} = {last.get(k)}")
    if not last.get("n_right_target"):
        add("  ⚠ 右臂目标一次都没发出去 —— 检查 send_action 走的是哪条话题")

    # ---- 底盘 ----
    add("\n[2] 底盘")
    wbc = [r for r in rows if "wbc_base_state" in r]
    if wbc:
        bs = [int(r["wbc_base_state"]) for r in wbc]
        # 只看"第一次锁上之后"是否再丢 —— 起始 UNLOCKED 是正常的待机状态，
        # 不能算"中途丢失"（之前的判据在这里误报过）。
        first_lock = bs.index(0) if 0 in bs else len(bs)
        after = bs[first_lock:]
        lost = [i for i, v in enumerate(after, start=first_lock) if v == 1]
        add(f"  首次 LOCKED 出现在采样 #{first_lock}"
            f"（t={wbc[first_lock]['t'] if first_lock < len(wbc) else float('nan'):.1f}s）")
        if first_lock == len(bs):
            add("  ❌ 全程没有 LOCKED —— BASE_LOCK 从未生效")
        elif not lost:
            add("  ✅ 锁上之后全程保持 LOCKED")
        else:
            add(f"  ❌ 锁上之后又丢失 {len(lost)} 次，首个在 #{lost[0]} "
                f"(t={wbc[lost[0]]['t']:.1f}s)")
        hj = {r.get("wbc_home_joint_reference_enabled") for r in wbc}
        add(f"  home_joint_reference_enabled: {hj}"
            + ("  ← 软约束开着（mu 很小，实测几乎不拽手，见 probe_infer_chain 的 D 阶段）"
               if True in hj else ""))
    wy = [r.get("q_waist_yaw_joint") for r in rows if r.get("q_waist_yaw_joint") is not None]
    if wy:
        add(f"  waist_yaw 范围: {min(wy):+.4f} .. {max(wy):+.4f} rad "
            f"(漂 {abs(max(wy)-min(wy)):.4f} rad)")
    for n in ("Left_Wheel_Joint", "Right_Wheel_Joint"):
        v = [r.get(f"q_{n}") for r in rows if r.get(f"q_{n}") is not None]
        if v:
            add(f"  {n} 范围: {min(v):+.6f} .. {max(v):+.6f} rad "
                f"(车轮在转说明底盘被推着走了)")

    # ---- 目标 vs 实测 ----
    # ---- 底盘/躯干位移（判断"底盘前移"是否修好的唯一硬指标）----
    add("\n[1b] 底盘/躯干位移（body_current_pose，world 系）")
    bp = [r for r in rows if "body_x" in r]
    if len(bp) > 3:
        B = np.array([[r["body_x"], r["body_y"], r["body_z"]] for r in bp])
        # 基准取"首次 LOCKED 那一帧"（推理真正开始），而不是采样起点
        lock_idx = [i for i, r in enumerate(rows)
                    if "body_x" in r and r.get("wbc_base_state") == 0]
        i0 = lock_idx[0] if lock_idx else 0
        B0 = np.array([rows[i0].get("body_x", 0.0), rows[i0].get("body_y", 0.0),
                       rows[i0].get("body_z", 0.0)])
        dev = B - B0
        add(f"  基准（首次 LOCKED 帧，t={rows[i0].get('t', 0):.1f}s）: "
            f"({B0[0]:.4f}, {B0[1]:.4f}, {B0[2]:.4f})")
        add(f"  末帧偏移: dx={dev[-1,0]*1000:+.2f}mm dy={dev[-1,1]*1000:+.2f}mm "
            f"dz={dev[-1,2]*1000:+.2f}mm")
        # **只看水平位移判断底盘**：body_current_pose 的 z 会随升降关节（LZ_mt/it）
        # 变化 80mrad 量级，那是正常的躯干升降、不是底盘移动（训练数据里也有）。
        # 用 3D 模长会把 100mm+ 的升降误判成"底盘跑了"。
        horiz = np.linalg.norm(dev[:, :2], axis=1)
        add(f"  水平位移: 末帧 {horiz[-1]*1000:.2f}mm  最大 {horiz.max()*1000:.2f}mm  "
            f"x 总行程 {(B[:,0].max()-B[:,0].min())*1000:.2f}mm")
        add(f"  （z 偏移 {dev[-1,2]*1000:+.1f}mm 是躯干升降，正常，不计入底盘判定）")
        if horiz.max() < 0.010:
            add("  ✅ 底盘锁定有效（水平位移 <10mm）")
        elif horiz.max() < 0.030:
            add("  ⚠ 底盘水平移动 10~30mm")
        else:
            add(f"  ❌ 底盘水平移动 {horiz.max()*1000:.0f}mm —— BASE_LOCK 未生效")
    else:
        add("  没有 body_current_pose 数据（话题未发布？）")

    add("\n[3] 目标 vs 实测（右臂）")
    tgt = [r for r in rows if "right_cmd_x" in r]
    cur = [r for r in rows if "right_cur_x" in r]
    if tgt and cur:
        T = np.array([[r["right_cmd_x"], r["right_cmd_y"], r["right_cmd_z"]] for r in tgt])
        C = np.array([[r["right_cur_x"], r["right_cur_y"], r["right_cur_z"]] for r in cur])
        n = min(len(T), len(C))
        T, C = T[:n], C[:n]
        tgt_path = float(np.linalg.norm(np.diff(T, axis=0), axis=1).sum())
        cur_path = float(np.linalg.norm(np.diff(C, axis=0), axis=1).sum())
        tgt_net = float(np.linalg.norm(T[-1] - T[0]))
        cur_net = float(np.linalg.norm(C[-1] - C[0]))
        add(f"  目标轨迹长度 {tgt_path*1000:.1f}mm  净位移 {tgt_net*1000:.1f}mm")
        add(f"  实测轨迹长度 {cur_path*1000:.1f}mm  净位移 {cur_net*1000:.1f}mm")
        reach = cur_path / tgt_path if tgt_path > 1e-6 else 0.0
        add(f"  reach_ratio = {reach:.2f}  (>0.8 说明运控跟得上)")
        # 目标是否自己往回走（策略想缩回）
        back = 0
        peak = 0.0
        for i in range(1, n):
            d = float(np.linalg.norm(T[i] - C[0]))
            peak = max(peak, d)
            if peak > 0.02 and d < peak - 0.02:
                back += 1
        add(f"  目标\"先远离起点又退回\"的帧数: {back}/{n}")
        if back > n * 0.1:
            add("  ⚠ 目标自己在缩回 —— 这是**策略输出**的问题，不是运控/底盘")
        # 用 z 轴看抓取/抬起
        add(f"  目标 z: {T[0,2]:.4f} → {T[:,2].max():.4f} → {T[-1,2]:.4f}")
        add(f"  实测 z: {C[0,2]:.4f} → {C[:,2].max():.4f} → {C[-1,2]:.4f}")

    # ---- FK 对照：target_echo 与 cmd 一致性 ----
    echo = [r for r in rows if "right_tgt_echo_x" in r]
    if echo and tgt:
        E = np.array([[r["right_tgt_echo_x"], r["right_tgt_echo_y"], r["right_tgt_echo_z"]] for r in echo])
        n = min(len(E), len(T))
        diff = np.linalg.norm(E[:n] - T[:n], axis=1)
        add(f"\n  /right_current_target 与 /right_target 差: "
            f"mean {diff.mean()*1000:.2f}mm max {diff.max()*1000:.1f}mm")
        if diff.mean() > 0.05:
            add("  ⚠ 运控内部目标与我们发的差很多 —— 可能被钳位/被 home 约束改写")

    # ---- 夹爪（关键：抓不住东西时唯一能定位的地方）----
    add("\n[4] 夹爪")
    for side in ("left", "right"):
        ck, pk, qk = f"grip_cmd_{side}", f"grip_pct_{side}", f"grip_q_{side}"
        cmd = [r.get(ck) for r in rows if r.get(ck) is not None]
        pct = [r.get(pk) for r in rows if r.get(pk) is not None]
        qv = [r.get(qk) for r in rows if r.get(qk) is not None]
        if not cmd and not qv:
            add(f"  {side}: 没有夹爪数据（话题未发布？）")
            continue
        line = f"  {side:5s}"
        if cmd:
            n0 = sum(1 for v in cmd if v == 0)
            line += f" 命令: 闭合(0) {n0}/{len(cmd)}"
        if pct:
            line += f"  percent 范围 {min(pct):.2f}..{max(pct):.2f}"
        if qv:
            line += f"  关节范围 {min(qv):+.4f}..{max(qv):+.4f}"
        add(line)
        # 命令闭合但关节没到闭合位 → 夹爪没执行
        if cmd and qv:
            n = min(len(cmd), len(qv))
            closed_cmd = [i for i in range(n) if cmd[i] == 0]
            if closed_cmd:
                q_at_closed = [qv[i] for i in closed_cmd]
                add(f"        命令闭合时关节值: mean {np.mean(q_at_closed):+.4f} "
                    f"（闭合限位约 -0.0200）")
                if np.mean(q_at_closed) > -0.005:
                    add("        ⚠ 命令是闭合但关节没到闭合位 —— 夹爪未执行闭合")
    if any(r.get("grip_cmd_left") is not None or r.get("grip_cmd_right") is not None
           for r in rows):
        add("  判读：若命令里**从未出现 0**，说明策略全程要求张开 —— 它没学到/没触发抓取。")
        add("        若命令出现 0 但关节没到 -0.02，说明运控/控制器没执行。")

    # ---- 【核心判据】合爪那一刻，策略想要在哪 vs 实际在哪 ----
    #
    # 这是区分"策略判错距离"和"运控没跟到"的唯一直接证据：
    #   策略想要的位置 = /right_target 的 x（我们直接把策略输出的 pos 发出去）
    #   实际位置       = /right_current_pose 的 x
    #   - 两者都 ≈1.99（真值合爪点）→ 策略和运控都对，问题在抓取本身
    #   - 想要 ≈1.99 但实际 ≈2.04 → **运控没跟到**（滞后/限速）
    #   - 想要 ≈2.04（还没到试管）→ **策略判错了距离**（图像/state 让它以为到了）
    add("\n[4b] 合爪时刻的位置归属（策略 vs 运控）")
    add("     真值参照：run_001 合爪在 right tcp x=1.9877；试管中心 x≈1.880")
    for side in ("right", "left"):
        ck = f"grip_cmd_{side}"
        seq = [(i, r) for i, r in enumerate(rows) if r.get(ck) is not None]
        if not seq:
            continue
        # 找"闭合"状态起始点。
        # 注意：不能只找 1→0 的跳变 —— 若策略**从第一帧就是闭合**（我们这次
        # 左爪 901/901 全 0），那条路径会一条都找不到，误报"从未闭合"。
        # 正确做法：把"首个闭合采样"也算作一个事件。
        events = []
        prev = None
        for i, r in seq:
            v = r[ck]
            if v == 0 and (prev is None or prev == 1):
                events.append(i)
            prev = v
        if not events:
            add(f"  {side}: 夹爪命令**从未闭合** —— 策略完全没有触发抓取")
            continue
        xs_all = [r.get(f"{side}_cur_x") for _, r in seq if r.get(f"{side}_cur_x") is not None]
        add(f"  {side}: 共 {len(events)} 次合爪起始（含首帧即为闭合的情况），逐个看：")
        for k, i in enumerate(events[:4], 1):
            r = rows[i]
            want_x = r.get(f"{side}_cmd_x")
            real_x = r.get(f"{side}_cur_x")
            if want_x is None or real_x is None:
                add(f"    #{k}: 缺位置数据")
                continue
            gap = real_x - want_x
            if want_x < 1.945 and real_x > 2.00:
                verdict = "← 策略想再往里，运控没跟到（运控滞后）"
            elif want_x > 2.00:
                verdict = "← 策略自己就判早了（目标还没到试管就合爪）"
            elif abs(gap) < 0.02:
                verdict = "← 策略与运控一致，位置已到"
            else:
                verdict = ""
            add(f"    #{k} t={r.get('t', 0):5.1f}s  "
                f"策略想要 x={want_x:.4f}  实际 x={real_x:.4f}  "
                f"差 {gap*1000:+.1f}mm  {verdict}")
        wt = [r.get(f"{side}_cmd_x") for _, r in seq if r.get(f"{side}_cmd_x") is not None]
        if xs_all:
            add(f"    实际 x 最小 {min(xs_all):.4f}（需 ≤1.99 才够到）")
        if wt:
            add(f"    目标 x 最小 {min(wt):.4f}（策略最远想到哪）")

    # ---- 图像 ----
    add("\n[5] 图像")
    add(f"  已保存 {len(img_steps)} 张拼图: {out_dir/'frames'}")
    add("  用文件管理器按顺序翻看 frames/m*.png，检查：")
    add("    - 试管在不在画面里、位置是否符合预期")
    add("    - 抓取瞬间夹爪与试管是否对齐（指尖应在试管两侧）")
    add("    - 是否有明显亮度/构图突变（分布外）")


    # ---- 关节力矩（HOME 约束线索）----
    add("\n[6] 关节力矩（判断是否有把手臂拽向 HOME 的持续力）")
    for n in ("waist_yaw_joint", "waist_pitch_joint"):
        v = [r.get(f"tau_{n}") for r in rows if r.get(f"tau_{n}") is not None]
        if v:
            add(f"  tau_{n}: mean {np.mean(v):+.3f} std {np.std(v):.3f}")

    return "\n".join(lines)


# ------------------------------------------------------------------ main


def main() -> int:
    ap = argparse.ArgumentParser(description="推理复盘探针")
    ap.add_argument("--seconds", type=float, default=60.0, help="采多久")
    ap.add_argument("--hz", type=float, default=20.0, help="采样频率")
    ap.add_argument("--every-img", type=int, default=15,
                    help="每多少帧存一张三路拼图")
    ap.add_argument("--out", default="", help="输出目录（默认 /tmp/sdg_infer_probe/<ts>）")
    args = ap.parse_args()

    import rclpy

    out_dir = (Path(args.out).expanduser() if args.out
               else OUT_ROOT / str(int(time.time())))
    (out_dir / "frames").mkdir(parents=True, exist_ok=True)
    _log(f"输出: {out_dir}")

    rclpy.init()
    node = rclpy.create_node("sdg_infer_probe")
    col = Collector(node)
    rows: list[dict] = []
    img_steps: list[int] = []

    period = 1.0 / max(args.hz, 1.0)
    t0 = time.monotonic()
    i = 0
    try:
        while time.monotonic() - t0 < args.seconds:
            step_t = time.monotonic()
            rclpy.spin_once(node, timeout_sec=0.0)
            row = col.snapshot(time.monotonic() - t0)
            rows.append(row)
            if i % max(1, args.every_img) == 0:
                imgs = {k.replace("img_", ""): v
                        for k, v in col.latest.items() if k.startswith("img_")}
                if imgs:
                    p = out_dir / "frames" / f"m{i:05d}.png"
                    try:
                        montage(imgs, p, step=i)
                        img_steps.append(i)
                    except Exception as exc:  # noqa: BLE001
                        _log(f"WARN 存图失败: {exc}")
            i += 1
            left = period - (time.monotonic() - step_t)
            if left > 0:
                time.sleep(left)
            if i % int(args.hz * 5) == 0:
                _log(f"  {i} 帧 / {time.monotonic()-t0:.0f}s")
    except KeyboardInterrupt:
        _log("中断")
    finally:
        node.destroy_node()
        rclpy.shutdown()

    # 落盘
    if rows:
        keys = sorted({k for r in rows for k in r})
        with (out_dir / "samples.csv").open("w", newline="", encoding="utf-8") as fh:
            w = csv.DictWriter(fh, fieldnames=keys)
            w.writeheader()
            for r in rows:
                w.writerow(r)
    report = analyse(rows, out_dir, img_steps)
    (out_dir / "report.txt").write_text(report, encoding="utf-8")
    print("\n" + "=" * 70)
    print(report)
    print("=" * 70)
    print(f"\n全部输出: {out_dir}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
