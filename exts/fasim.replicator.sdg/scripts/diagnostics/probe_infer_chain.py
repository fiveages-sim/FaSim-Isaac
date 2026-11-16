#!/usr/bin/env python3
"""推理链路探查：从「刚起仿真、运控 HOLD 待命」的干净状态开始，
量化推理与编排采集之间的工况差异。

## 为什么要有它
现象：推理时右手伸向试管，快到时又缩回，反复来；底盘还缓慢前移。
人眼只能看到"来回动"，看不到：
  - 到底是**策略目标**在往回走，还是**运控没跟到**
  - BASE_LOCK / HOME_JOINT 这些约束在不在拽手
  - 底盘是被反作用力推的，还是 WBC 自己在规划移动

本脚本**只读 + 少量已知安全的 mode 切换**，按阶段采样并落盘，跑完自判。

## 四个阶段（默认全跑）
| 阶段 | 做什么 | 目的 |
|---|---|---|
| A `hold` | 什么都不发，采 3 秒 | 基线：运控待命时的漂移量 |
| B `ocs2` | 切 OCS2，采 3 秒 | 不锁底盘进 OCS2 会不会自己跑 |
| C `lock` | 发 BASE_LOCK，采 3 秒 | 锁定后底盘是否真不动 |
| D `probe` | 只朝 +x 推右手 3cm 再回原位 | 测**跟随率**与**HOME 约束回拉** |

D 阶段是核心：给一个明确的 3cm 阶跃，看
  - 实测走了多少（follow_ratio）
  - **命令撤销后手会不会自己回到 HOME**（这是 HOME_JOINT 约束的直接证据）

## 用法
```bash
# 仿真已开、运控 HOLD 待命时：
cd ~/lerobot_ros2 && source /opt/ros/jazzy/setup.bash
source .venv-train/bin/activate
python ~/FaSim-Isaac/exts/fasim.replicator.sdg/scripts/probe_infer_chain.py

# 只看不动（不发任何指令）：
python ~/FaSim-Isaac/exts/fasim.replicator.sdg/scripts/probe_infer_chain.py --readonly

# 跑完自动回 HOLD 并解锁（默认会）
```

产物：`/tmp/sdg_chain_probe/<ts>/` 下 `samples.csv` + `report.txt`。
"""

from __future__ import annotations

import argparse
import csv
import time
from pathlib import Path

import numpy as np

OUT_ROOT = Path("/tmp/sdg_chain_probe")


def _log(m: str) -> None:
    print(f"[probe] {m}", flush=True)


class Chain:
    def __init__(self):
        import rclpy
        from geometry_msgs.msg import Pose, PoseStamped
        from sensor_msgs.msg import JointState
        from std_msgs.msg import Int32, String
        from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy

        self.rclpy = rclpy
        rclpy.init()
        self.node = rclpy.create_node("sdg_chain_probe")
        n = self.node

        latched = QoSProfile(depth=1)
        latched.durability = DurabilityPolicy.TRANSIENT_LOCAL
        latched.reliability = ReliabilityPolicy.RELIABLE

        self.latest: dict = {}
        # ⚠ 类型必须对：运控发布的 *_current_pose / *_current_target 是
        # PoseStamped；只有我们下发的 /<side>_target 是纯 Pose。
        # 用错类型订阅不会报错，只是**一条都收不到**（曾经因此整段 D 阶段被跳过）。
        n.create_subscription(PoseStamped, "/left_current_pose", lambda m: self.latest.__setitem__("left_cur", m), 10)
        n.create_subscription(PoseStamped, "/right_current_pose", lambda m: self.latest.__setitem__("right_cur", m), 10)
        n.create_subscription(PoseStamped, "/left_current_target", lambda m: self.latest.__setitem__("left_echo", m), 10)
        n.create_subscription(PoseStamped, "/right_current_target", lambda m: self.latest.__setitem__("right_echo", m), 10)
        n.create_subscription(PoseStamped, "/body_current_pose", lambda m: self.latest.__setitem__("body_cur", m), 10)
        n.create_subscription(JointState, "/joint_states", self._js, 10)
        n.create_subscription(Int32, "/fsm_state", lambda m: self.latest.__setitem__("fsm", m), latched)

        self.pub_fsm = n.create_publisher(Int32, "/fsm_command", 10)
        self.pub_mode = n.create_publisher(String, "/mode_command", 10)
        self.pub_left = n.create_publisher(Pose, "/left_target", 10)
        self.pub_right = n.create_publisher(Pose, "/right_target", 10)

        try:
            from rosidl_runtime_py.utilities import get_message
            cls = get_message("arms_ros2_control_msgs/msg/WbcCurrentState")
            n.create_subscription(cls, "/ocs2_wbc_controller/current_state",
                                  lambda m: self.latest.__setitem__("wbc", m), latched)
        except Exception as e:  # noqa: BLE001
            _log(f"WARN WBC 状态订阅失败: {e}")

    def _js(self, m):
        self.latest["joint_names"] = list(m.name)
        self.latest["joint_pos"] = np.asarray(m.position, dtype=np.float64)

    def spin(self, sec: float, hz: float = 20.0) -> None:
        t0 = time.monotonic()
        while time.monotonic() - t0 < sec:
            self.rclpy.spin_once(self.node, timeout_sec=0.0)
            time.sleep(max(0.0, 1.0 / hz - 0.001))

    @staticmethod
    def pose(m) -> np.ndarray:
        """取位置。兼容 Pose 与 PoseStamped（运控发的是 PoseStamped）。"""
        p = getattr(m, "pose", m).position
        return np.array([p.x, p.y, p.z], dtype=np.float64)

    def row(self, t: float, phase: str) -> dict:
        d: dict = {"t": round(t, 4), "phase": phase}
        for k in ("left_cur", "right_cur", "left_echo", "right_echo", "body_cur"):
            m = self.latest.get(k)
            if m is not None:
                p = self.pose(m)
                for i, ax in enumerate("xyz"):
                    d[f"{k}_{ax}"] = round(float(p[i]), 5)
        wbc = self.latest.get("wbc")
        if wbc is not None:
            d["base_state"] = int(getattr(wbc, "base_state", -1))
            d["body_state"] = int(getattr(wbc, "body_state", -1))
            d["left_arm_state"] = int(getattr(wbc, "left_arm_state", -1))
            d["right_arm_state"] = int(getattr(wbc, "right_arm_state", -1))
            d["home_joint"] = int(bool(getattr(wbc, "home_joint_reference_enabled", False)))
        fsm = self.latest.get("fsm")
        if fsm is not None:
            d["fsm"] = int(getattr(fsm, "data", -1))
        jn = self.latest.get("joint_names") or []
        jp = self.latest.get("joint_pos")
        if jp is not None:
            for name in ("waist_yaw_joint", "waist_pitch_joint",
                         "Left_Wheel_Joint", "Right_Wheel_Joint"):
                if name in jn:
                    i = jn.index(name)
                    if i < len(jp):
                        d[f"q_{name}"] = round(float(jp[i]), 6)
        return d

    def set_fsm(self, code: int) -> None:
        from std_msgs.msg import Int32
        m = Int32(); m.data = int(code)
        self.pub_fsm.publish(m)
        _log(f"→ FSM={code}")
        self.spin(0.35)

    def set_mode(self, name: str) -> None:
        from std_msgs.msg import String
        m = String(); m.data = name
        self.pub_mode.publish(m)
        _log(f"→ mode {name}")
        self.spin(0.35)

    def send_right(self, xyz: np.ndarray, quat: np.ndarray) -> None:
        from geometry_msgs.msg import Pose
        m = Pose()
        m.position.x, m.position.y, m.position.z = (float(v) for v in xyz)
        m.orientation.x, m.orientation.y, m.orientation.z, m.orientation.w = (
            float(v) for v in quat)
        self.pub_right.publish(m)

    def send_left(self, xyz: np.ndarray, quat: np.ndarray) -> None:
        from geometry_msgs.msg import Pose
        m = Pose()
        m.position.x, m.position.y, m.position.z = (float(v) for v in xyz)
        m.orientation.x, m.orientation.y, m.orientation.z, m.orientation.w = (
            float(v) for v in quat)
        self.pub_left.publish(m)

    def shutdown(self) -> None:
        try:
            self.node.destroy_node()
            self.rclpy.shutdown()
        except Exception:  # noqa: BLE001
            pass


def _quat_of(m) -> np.ndarray:
    """从 PoseStamped 取姿态（运控发布的是 PoseStamped，见订阅处的说明）。"""
    p = getattr(m, "pose", m)     # 兼容 Pose / PoseStamped
    o = p.orientation
    return np.array([o.x, o.y, o.z, o.w], dtype=np.float64)


def analyse(rows: list[dict], phases_seen: list[str]) -> str:
    out: list[str] = []
    a = out.append
    if not rows:
        return "没采到数据"

    a("=" * 72)
    a("推理链路探查报告")
    a("=" * 72)

    # ---- 全局：基线与底盘 ----
    a("\n[1] 底盘 / 约束状态")
    for ph in phases_seen:
        sub = [r for r in rows if r["phase"] == ph]
        if not sub:
            continue
        base = {r.get("base_state") for r in sub if "base_state" in r}
        hj = {r.get("home_joint") for r in sub if "home_joint" in r}
        fsm = {r.get("fsm") for r in sub if "fsm" in r}
        a(f"  {ph:8s} fsm={sorted(x for x in fsm if x is not None)} "
          f"base_state={sorted(x for x in base if x is not None)} "
          f"home_joint={sorted(x for x in hj if x is not None)}")

    for ph in phases_seen:
        sub = [r for r in rows if r["phase"] == ph and "body_cur_x" in r]
        if len(sub) < 3:
            continue
        P = np.array([[r["body_cur_x"], r["body_cur_y"], r["body_cur_z"]] for r in sub])
        drift = float(np.linalg.norm(P[-1] - P[0]))
        path = float(np.linalg.norm(np.diff(P, axis=0), axis=1).sum())
        a(f"  {ph:8s} body 净漂移 {drift*1000:6.2f}mm  轨迹长 {path*1000:6.2f}mm")

    # ---- D 阶段：跟随率 + HOME 回拉 ----
    a("\n[2] 阶跃跟随（D 阶段）")
    d = [r for r in rows if r["phase"] == "push"]
    ret = [r for r in rows if r["phase"] == "release"]
    if len(d) > 5:
        C = np.array([[r["right_cur_x"], r["right_cur_y"], r["right_cur_z"]]
                      for r in d if "right_cur_x" in r])
        E = np.array([[r["right_echo_x"], r["right_echo_y"], r["right_echo_z"]]
                      for r in d if "right_echo_x" in r])
        if len(C) > 3:
            a(f"  推送段: 实测净位移 {float(np.linalg.norm(C[-1]-C[0]))*1000:6.1f}mm")
        if len(E) > 3:
            a(f"          运控内部目标净位移 {float(np.linalg.norm(E[-1]-E[0]))*1000:6.1f}mm")
        if len(C) > 3 and len(E) > 3:
            ratio = float(np.linalg.norm(C[-1]-C[0])) / max(
                float(np.linalg.norm(E[-1]-E[0])), 1e-9)
            a(f"          跟随率 = {ratio:.2f}  (≈1 正常；明显 <0.8 说明被约束/限速)")
    if len(ret) > 5:
        C = np.array([[r["right_cur_x"], r["right_cur_y"], r["right_cur_z"]]
                      for r in ret if "right_cur_x" in r])
        if len(C) > 3:
            back = float(np.linalg.norm(C[-1] - C[0]))
            a(f"  撤销段: 手自行移动 {back*1000:6.1f}mm")
            if back > 0.01:
                a("  ⚠ 命令已撤但仍移动 >10mm —— HOME 软约束在回拉（或残余跟踪）")
            else:
                a("  ✅ 撤销后基本静止 —— 无强 HOME 回拉")

    # ---- 关节：waist / 轮 ----
    a("\n[3] 关节（判断底盘是否被推着走）")
    for name in ("q_waist_yaw_joint", "q_Left_Wheel_Joint", "q_Right_Wheel_Joint"):
        v = [r.get(name) for r in rows if r.get(name) is not None]
        if v:
            a(f"  {name:22s} 范围 {min(v):+.6f} .. {max(v):+.6f}  "
              f"(跨度 {abs(max(v)-min(v)):.6f})")

    # ---- 结论 ----
    a("\n[4] 结论")
    base_locked = [r.get("base_state") for r in rows if r.get("base_state") == 0]
    a(f"  出现过 BASE_LOCKED 的采样数: {len(base_locked)}")
    hj = [r.get("home_joint") for r in rows if r.get("home_joint") == 1]
    a(f"  home_joint_reference 为 True 的采样数: {len(hj)} / {len(rows)}")
    if hj:
        a("  ⇒ HOME 软约束全程开启。它会把手臂往**全零姿态**拉；")
        a("    策略目标离开零点越远，回拉越明显 —— 这是\"伸出去又缩回\"的候选原因之一。")
    return "\n".join(out)


def main() -> int:
    ap = argparse.ArgumentParser(description="推理链路探查")
    ap.add_argument("--readonly", action="store_true", help="只读，不发任何指令")
    ap.add_argument("--push-mm", type=float, default=30.0, help="D 阶段沿 +x 推多少毫米")
    ap.add_argument("--hold-sec", type=float, default=3.0)
    ap.add_argument("--out", default="")
    args = ap.parse_args()

    out_dir = (Path(args.out).expanduser() if args.out
               else OUT_ROOT / str(int(time.time())))
    out_dir.mkdir(parents=True, exist_ok=True)
    _log(f"输出 {out_dir}")

    c = Chain()
    rows: list[dict] = []
    phases_seen: list[str] = []
    t0 = time.monotonic()

    def record(phase: str, sec: float) -> None:
        if phase not in phases_seen:
            phases_seen.append(phase)
        end = time.monotonic() + sec
        while time.monotonic() < end:
            c.rclpy.spin_once(c.node, timeout_sec=0.0)
            rows.append(c.row(time.monotonic() - t0, phase))
            time.sleep(0.05)

    try:
        record("hold", args.hold_sec)

        if not args.readonly:
            c.set_fsm(3)          # OCS2（send_fsm_command 会先自动 HOLD）
            record("ocs2", args.hold_sec)

            c.set_mode("BASE_LOCK")
            record("lock", args.hold_sec)

            # ---- D: 阶跃 ----
            m = c.latest.get("right_cur")
            if m is None:
                _log("WARN 没有 right_current_pose —— 无法做阶跃测试。")
                _log("     排查：")
                _log("       1) ros2 topic info /right_current_pose -v  看类型是否 PoseStamped")
                _log("       2) 运控是否在 OCS2（fsm=3）")
                _log("       3) ros2 topic echo /right_current_pose --once 是否出数据")
                _log("     本脚本的 A/B/C 阶段数据仍然有效，D 阶段跳过。")
            else:
                p0 = c.pose(m)
                q0 = _quat_of(m)
                _log(f"起点 {p0.round(4)}")
                tgt = p0 + np.array([args.push_mm / 1000.0, 0.0, 0.0])

                # 推送 1.5s
                phase_in = "push"
                phases_seen.append(phase_in) if phase_in not in phases_seen else None
                end = time.monotonic() + 1.5
                while time.monotonic() < end:
                    c.send_right(tgt, q0)
                    c.rclpy.spin_once(c.node, timeout_sec=0.0)
                    rows.append(c.row(time.monotonic() - t0, phase_in))
                    time.sleep(0.05)

                # 撤销：发回起点，观察是否"自己还在动"
                record("release", 2.0)

            # ---- 复位 ----
            _log("复位：回 HOLD")
            c.set_fsm(2)
            c.spin(0.5)
    except KeyboardInterrupt:
        _log("中断")
    finally:
        c.shutdown()

    if rows:
        keys = sorted({k for r in rows for k in r})
        with (out_dir / "samples.csv").open("w", newline="", encoding="utf-8") as fh:
            w = csv.DictWriter(fh, fieldnames=keys)
            w.writeheader()
            w.writerows(rows)
    rep = analyse(rows, phases_seen)
    (out_dir / "report.txt").write_text(rep, encoding="utf-8")
    print("\n" + rep + f"\n\n输出: {out_dir}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
