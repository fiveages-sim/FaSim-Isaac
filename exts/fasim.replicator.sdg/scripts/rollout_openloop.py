#!/usr/bin/env python3
"""开环策略回放：喂一张图，把策略的整条 chunk 一次性算完，直接驱动运控。

## 为什么需要
闭环失败时有两种可能：
  A) 策略输出本身就是错的（轨迹不对）
  B) 策略输出对，但闭环里观测漂移把它带偏了

本脚本用**纯开环**把 A/B 分开：
  - 只读**一次**观测（图像 + state）
  - 一次性算出 chunk_size 个动作（默认 100 帧 = 5 秒）
  - 按时间顺序逐帧下发，**中途不回读观测**
  - 走的是和闭环完全相同的通道（`/<side>_target` → OCS2 IK）

于是机器人执行的就是"策略在 t=0 那一刻想要的完整轨迹"。
轨迹对不对，肉眼一看就知道。

## 和闭环的区别
| | 闭环 inference.py | 本脚本 |
|---|---|---|
| 观测 | 每 N 帧回读 | **只读第 1 帧** |
| 前向次数 | 多次 | **1 次** |
| 用途 | 测成功率 | **测轨迹正确性** |

## 用法
```bash
# 仿真已开、机器人待命
cd ~/lerobot_ros2 && source /opt/ros/jazzy/setup.bash && source .venv-train/bin/activate
python ~/FaSim-Isaac/exts/fasim.replicator.sdg/scripts/rollout_openloop.py \
    --dataset ~/lerobot_datasets/tube_grip \
    --train-config ~/act_runs/tube_grip_30k/checkpoints/last/pretrained_model \
    --frames 100 --hz 20

# 只算不动（先看轨迹数值，确认安全）
python ~/FaSim-Isaac/exts/fasim.replicator.sdg/scripts/rollout_openloop.py --dry-run
```

产物写到 `/tmp/sdg_openloop/<ts>/`：
  - `trajectory.csv`  逐帧的目标位姿/夹爪
  - `report.txt`      与训练数据的对比 + 自动判定
  - `montage.png`     首帧三路相机（确认输入正确）
"""

from __future__ import annotations

import argparse
import csv
import json
import time
from pathlib import Path

import numpy as np

OUT_ROOT = Path("/tmp/sdg_openloop")

#: 训练数据的参照值（由 `--ref-dataset` 自动统计，也可手工覆盖）
DEFAULT_REF = {
    "grasp_x_mean": 1.9722,
    "grasp_x_p5": 1.9267,
    "grasp_x_p95": 1.9898,
    "grasp_z_mean": 0.9336,
    "grasp_frame_median": 53,
}


def _log(m: str) -> None:
    print(f"[openloop] {m}", flush=True)


# ---------------------------------------------------------------- 策略侧


def load_policy(ckpt: Path, device: str = "cuda"):
    from lerobot.policies.act.modeling_act import ACTPolicy
    from lerobot.processor.pipeline import PolicyProcessorPipeline

    pol = ACTPolicy.from_pretrained(str(ckpt))
    pol.to(device)
    pol.eval()
    pre = PolicyProcessorPipeline.from_pretrained(
        pretrained_model_name_or_path=str(ckpt),
        config_filename="policy_preprocessor.json", to_transition=None)
    post = PolicyProcessorPipeline.from_pretrained(
        pretrained_model_name_or_path=str(ckpt),
        config_filename="policy_postprocessor.json", to_transition=None)
    return pol, pre, post


def build_batch(meta, obs: dict, cam_map: dict, device: str, core=None):
    """把 robot 观测 → 策略输入 batch。

    与 ``core.py`` 的 ``infer_action_vec`` **同口径**：state 按数据集 names 顺序
    取（缺失填 0），相机走 ``core._camera_alias_map`` 的别名映射（
    ``head``→``head_camera``、``wrist_left``→``left_hand_camera`` …），
    再 resize 到数据集 feature shape。
    """
    import torch
    import cv2

    names = meta.features.get("observation.state", {}).get("names", [])
    state = np.zeros(len(names), dtype=np.float32)
    missing = []
    for i, n in enumerate(names):
        for cand in (n, f"{n}.pos"):
            if cand in obs:
                state[i] = float(obs[cand])
                break
        else:
            missing.append(n)
    if missing:
        _log(f"WARN {len(missing)}/{len(names)} 个 state 维度缺失，填 0"
             f"（示例 {missing[:4]}）")

    batch = {"observation.state": torch.from_numpy(state).unsqueeze(0).to(device)}

    # 相机别名：优先用 core 的实现（已验证），退化到手工 map
    alias = None
    if core is not None:
        try:
            alias = core._camera_alias_map(meta)  # noqa: SLF001
            _log(f"相机别名（core）: {alias}")
        except Exception as exc:  # noqa: BLE001
            _log(f"WARN core 别名失败，用内置: {exc}")
    if alias is None:
        alias = {k.replace("observation.images.", ""): cam_map.get(
            k.replace("observation.images.", ""),
            k.replace("observation.images.", "")) for k in meta.camera_keys}

    for cam_key in meta.camera_keys:
        ds_name = cam_key.replace("observation.images.", "")
        robot_key = alias.get(ds_name, ds_name)
        img = None
        # 依次尝试各种可能的 obs 键写法（有深度的相机拆成 <key>.rgb）
        for cand in (f"{robot_key}.rgb", robot_key,
                     f"{robot_key}_camera.rgb", f"{robot_key}_camera",
                     f"{ds_name}.rgb", ds_name):
            v = obs.get(cand)
            if v is None:
                continue
            img = v.get("rgb") if isinstance(v, dict) else v
            if img is not None:
                break
        if img is None:
            raise RuntimeError(f"找不到相机 {ds_name}（robot key={robot_key}）")
        img = np.asarray(img, dtype=np.uint8)
        feat = meta.features.get(cam_key) or {}
        shp = feat.get("shape") or []
        if len(shp) >= 2:
            h, w = int(shp[0]), int(shp[1])
            if img.shape[:2] != (h, w):
                img = cv2.resize(img, (w, h), interpolation=cv2.INTER_AREA)
        t = torch.from_numpy(img[..., :3]).permute(2, 0, 1).float() / 255.0
        batch[cam_key] = t.unsqueeze(0).to(device)
    return batch


def predict_chunk(pol, pre, post, batch, n_frames: int) -> np.ndarray:
    import torch

    b = pre(batch)
    with torch.no_grad():
        ch = pol.predict_action_chunk(b).squeeze(0)
    pol.reset()
    out = post({"action": ch})
    a = out.get("action", ch) if isinstance(out, dict) else out
    arr = np.asarray(a.detach().cpu(), dtype=np.float64)
    return arr[:n_frames]


# ---------------------------------------------------------------- 分析


def analyse(traj: np.ndarray, names: list[str], ref: dict) -> str:
    """与训练数据参照对比，回答"轨迹对不对"。"""
    L = []
    add = L.append
    idx = {n: i for i, n in enumerate(names)}
    rx = traj[:, idx["right_ee.pos.x"]]
    rz = traj[:, idx["right_ee.pos.z"]]
    ry = traj[:, idx["right_ee.pos.y"]]
    g = traj[:, idx["right_gripper.pos"]] if "right_gripper.pos" in idx else None

    add("=" * 72)
    add("开环轨迹分析（策略在 t=0 那一刻想要的完整动作序列）")
    add("=" * 72)
    add(f"\n右臂 x: {rx[0]:.4f} → min {rx.min():.4f} → {rx[-1]:.4f}")
    add(f"右臂 z: {rz[0]:.4f} → max {rz.max():.4f} → {rz[-1]:.4f}")
    add(f"右臂 y: {ry[0]:.4f} → {ry.min():.4f}..{ry.max():.4f} → {ry[-1]:.4f}")

    ok = True
    add("\n[1] 是否够到试管（训练合爪点 x）")
    add(f"    训练参照: mean {ref['grasp_x_mean']:.4f}  "
        f"5~95% [{ref['grasp_x_p5']:.4f}, {ref['grasp_x_p95']:.4f}]")
    add(f"    本次最小 x = {rx.min():.4f}  差 {(rx.min()-ref['grasp_x_mean'])*1000:+.1f}mm")
    if rx.min() > ref["grasp_x_p95"] + 0.035:
        add(f"    ❌ 没够到（比训练上限还远 {(rx.min()-ref['grasp_x_p95'])*1000:.0f}mm）"
            " → 策略输出本身就不对")
        ok = False
    else:
        add("    ✅ 到达了训练分布内的合爪位置"
            "（容差按夹爪开口放宽，Dex1 开口 49mm、试管约 13mm）")

    if g is not None:
        add("\n[2] 夹爪时序")
        gc = np.where(g < 0.5)[0]
        if len(gc):
            f0 = int(gc[0])
            add(f"    首次合爪 @ 帧 {f0}（训练中位 {ref['grasp_frame_median']}）")
            add(f"    合爪时 x = {rx[f0]:.4f}（训练合爪点 {ref['grasp_x_mean']:.4f}）")
            add(f"    合爪时 z = {rz[f0]:.4f}（训练 {ref['grasp_z_mean']:.4f}）")
            dx = (rx[f0] - ref["grasp_x_mean"]) * 1000
            dz = (rz[f0] - ref["grasp_z_mean"]) * 1000
            # 容差取夹爪开口量级，不是训练 std。
            # 实测：Δx=+27mm 的轨迹**真的抓起来了** —— Dex1 开口 49mm、
            # 试管约 13mm 粗，横向 25–30mm 误差仍能夹住。原先用 25mm 判失败
            # 是错的（把成功案例报成"瞄错目标"）。
            TOL_X_MM, TOL_Z_MM = 35.0, 20.0
            if abs(dx) < TOL_X_MM and abs(dz) < TOL_Z_MM:
                add(f"    ✅ 合爪位置可用（Δx={dx:+.0f}mm Δz={dz:+.0f}mm，"
                    f"容差 ±{TOL_X_MM:.0f}/±{TOL_Z_MM:.0f}mm）")
            else:
                add(f"    ❌ 合爪位置偏了（Δx={dx:+.0f}mm Δz={dz:+.0f}mm，"
                    f"超容差）→ 可能瞄错目标")
                ok = False
            # 合爪后是否保持闭合
            after = g[f0:]
            reopen = int((after > 0.5).sum())
            add(f"    合爪后张开的帧数: {reopen}/{len(after)}"
                + ("（✅ 保持闭合）" if reopen < len(after) * 0.1 else "（❌ 抖动）"))
        else:
            add("    ❌ 全程张开 —— 策略没触发抓取")
            ok = False

    # 单调性：接近段是否单调
    add("\n[3] 接近段单调性（判断是否在来回摆）")
    n_first = min(len(rx), 60)
    d = np.diff(rx[:n_first])
    back = int((d > 0.003).sum())
    add(f"    前 {n_first} 帧里 x 反向（远离试管）次数: {back}")
    if back > 3:
        add("    ⚠ 有来回摆动 —— 策略输出的轨迹不单调")
    else:
        add("    ✅ 单调接近")

    add("\n" + "=" * 72)
    add("结论: " + ("✅ 轨迹正确 —— 闭环失败是观测漂移导致（加数据/改执行方式）"
                  if ok else "❌ 轨迹本身就不对 —— 策略没学好，需要更多数据或更久训练"))
    add("=" * 72)
    return "\n".join(L)


# ---------------------------------------------------------------- main


def main() -> int:
    ap = argparse.ArgumentParser(description="开环策略回放")
    ap.add_argument("--dataset", default=str(Path.home() / "lerobot_datasets/tube_grip"))
    ap.add_argument("--train-config",
                    default=str(Path.home() / "act_runs/tube_grip_30k/checkpoints/last/pretrained_model"))
    ap.add_argument("--frames", type=int, default=100, help="执行多少帧（≤ chunk_size）")
    ap.add_argument("--hz", type=float, default=20.0)
    ap.add_argument("--speed", type=float, default=1.0, help="播放倍速（1=训练速度）")
    ap.add_argument("--dry-run", action="store_true", help="只算不发指令")
    ap.add_argument("--out", default="")
    args = ap.parse_args()

    out = Path(args.out) if args.out else OUT_ROOT / str(int(time.time()))
    out.mkdir(parents=True, exist_ok=True)
    _log(f"输出 {out}")

    # ---- 策略 ----
    from lerobot.datasets.lerobot_dataset import LeRobotDatasetMetadata
    ds_root = Path(args.dataset).expanduser().resolve()
    meta = LeRobotDatasetMetadata(ds_root.name, root=str(ds_root))
    action_names = meta.features.get("action", {}).get("names", [])
    ckpt = Path(args.train_config).expanduser()
    if ckpt.name != "pretrained_model":
        ckpt = ckpt / "pretrained_model"
    pol, pre, post = load_policy(ckpt)
    _log(f"策略已加载，action 维度 {len(action_names)}")

    # ---- 机器人 ----
    robot = None
    if not args.dry_run:
        import sys
        sys.path.insert(0, str(Path.home() / "lerobot_ros2/examples/IsaacSim"))
        sys.path.insert(0, str(Path.home() / "lerobot_ros2/examples/IsaacSim/common"))
        from pathlib import Path as _P
        import importlib.util
        isaac_dir = _P.home() / "lerobot_ros2/examples/IsaacSim"
        spec = importlib.util.spec_from_file_location("core_ol", isaac_dir / "online_infer/core.py")
        core = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(core)

        from robot_action_composer.discovery.registry_loader import (
            load_lerobot_profile, load_motion_entries, resolve_robot_profile_dir)
        from dataclasses import replace
        ws = isaac_dir
        reg = load_motion_entries(ws)
        entry = reg["unitree_g1d"]
        motion_cfg = entry["motion_cfg"]
        lerobot_cfg = load_lerobot_profile(resolve_robot_profile_dir(ws, entry), workspace_dir=ws)
        core.MOTION_CFG = motion_cfg
        core.LEROBOT_CFG = lerobot_cfg
        core.ROBOT_CFG = motion_cfg
        core.FPS = int(args.hz)
        robot = core.build_robot()
        robot.connect()
        _log("机器人已连接")
        # 与 inference.py **完全相同**的运控准备（HOLD → 模式 → 播种 → OCS2
        # → 使能 → settle → 终检）。抽成 bringup_control 就是为了两条路径不漂移。
        try:
            iface = robot.ros2_interface
            iface.config.auto_switch_fsm_before_control = False
            iface._auto_switch_fsm_before_control = False  # noqa: SLF001
        except Exception:  # noqa: BLE001
            pass
        _log("等待场景就绪（只读，不复位仿真）...")
        core._wait_for_scene_ready(robot, timeout_s=15.0)
        if not hasattr(args, "settle_s"):
            args.settle_s = 1.0
        _log("bringup：HOLD → 模式 → 播种 → OCS2 → 使能 → settle ...")
        ok = core.bringup_control(
            robot, args, core.SimTimeHelper(),
            motion_cfg=motion_cfg, want_lock=True)
        if not ok:
            _log("WARN 终检未完全通过，继续（可能仍可用）")
        _log("运控已就绪")

    # ---- 读一次观测 ----
    cam_map = {}
    if robot is None:
        # dry-run：用**数据集第 0 帧**当输入。
        # 依据：初始状态下推理与采集基本一致（已实测：同一 state 下
        # 数据集图 vs 实时图 输出差 0.57mm），所以这是合法的近似。
        from lerobot.datasets.lerobot_dataset import LeRobotDataset
        import torch
        ds = LeRobotDataset(repo_id=ds_root.name, root=str(ds_root),
                            video_backend="pyav")
        it = ds[0]
        batch = {}
        st = it["observation.state"]
        st = st.numpy() if hasattr(st, "numpy") else np.asarray(st)
        batch["observation.state"] = torch.as_tensor(
            st, dtype=torch.float32).reshape(1, -1).to("cuda")
        for cam_key in meta.camera_keys:
            v = it[cam_key]
            v = v.numpy() if hasattr(v, "numpy") else np.asarray(v)
            if v.ndim == 3 and v.shape[0] in (1, 3, 4):
                v = np.transpose(v, (1, 2, 0))
            if v.dtype != np.uint8:
                v = (np.clip(v, 0, 1) * 255).astype(np.uint8)
            t = torch.from_numpy(v[..., :3]).permute(2, 0, 1).float() / 255.0
            batch[cam_key] = t.unsqueeze(0).to("cuda")
        _log("dry-run：输入 = 数据集第 0 帧（图像 + state）")
    else:
        obs = robot.get_observation()
        try:
            cfg = core.LEROBOT_CFG
            for k in getattr(cfg, "cameras", {}):
                cam_map[k.replace("_camera", "")] = k
        except Exception:  # noqa: BLE001
            pass
        _log(f"观测键数 {len(obs)}，相机映射 {cam_map}")
        batch = build_batch(meta, obs, cam_map, "cuda", core=core)
    traj = predict_chunk(pol, pre, post, batch, args.frames)
    _log(f"chunk 已算出 {traj.shape}")

    # ---- 落盘 ----
    with (out / "trajectory.csv").open("w", newline="", encoding="utf-8") as fh:
        w = csv.writer(fh)
        w.writerow(["frame"] + action_names)
        for i, row in enumerate(traj):
            w.writerow([i] + [f"{v:.6f}" for v in row])

    # 参照值：优先从数据集统计
    ref = dict(DEFAULT_REF)
    try:
        import pandas as pd
        df = pd.read_parquet(next((ds_root / "data").rglob("*.parquet")))
        ep = df["episode_index"].values
        A = np.stack(df["action"].values)
        xs, zs, f0s = [], [], []
        for e in np.unique(ep):
            sel = np.where(ep == e)[0]
            a = A[sel]
            if a.shape[1] < 16:
                continue
            g = a[:, 15]
            # 抓取帧的定义要小心：上电 idle 值 0（两指并拢）与真正的闭合
            # (=关节到 -0.02) 在开关量上**都是 0**，所以不能直接取首个 0。
            # 正确做法：先找到首次「张开(1)」，之后的第一个 0 才是抓取。
            opened = np.where(g > 0.5)[0]
            if not len(opened):
                continue
            after = np.where(g[opened[0]:] < 0.5)[0]
            if not len(after):
                continue
            i0 = int(opened[0] + after[0])
            xs.append(float(a[i0, 7]))
            zs.append(float(a[i0, 9]))
            f0s.append(i0)
        if xs:
            ref["grasp_x_mean"] = float(np.mean(xs))
            ref["grasp_x_p5"] = float(np.percentile(xs, 5))
            ref["grasp_x_p95"] = float(np.percentile(xs, 95))
            ref["grasp_z_mean"] = float(np.mean(zs))
            ref["grasp_frame_median"] = int(np.median(f0s))
            _log(f"参照值已从数据集统计（{len(xs)} 条）："
                 f"合爪 x={ref['grasp_x_mean']:.4f} z={ref['grasp_z_mean']:.4f} "
                 f"帧={ref['grasp_frame_median']}")
    except Exception as exc:  # noqa: BLE001
        _log(f"参照值用默认（数据集统计失败: {exc}）")

    report = analyse(traj, action_names, ref)
    (out / "report.txt").write_text(report, encoding="utf-8")
    print("\n" + report)

    if args.dry_run:
        _log("--dry-run：未发送任何指令")
        _log(f"输出: {out}")
        return 0

    # ---- 执行 ----
    _log(f"开始执行 {len(traj)} 帧 @ {args.hz}Hz（开环，不回读观测）")
    period = 1.0 / args.hz
    try:
        for i, row in enumerate(traj):
            cmd = {n: float(row[j]) for j, n in enumerate(action_names)}
            # 夹爪二值化（与 core.py 一致）
            for key in ("left_gripper.pos", "right_gripper.pos"):
                if key in cmd:
                    cmd[key] = 0.0 if cmd[key] < 0.5 else 1.0
            robot.send_action(cmd)
            if i % 10 == 0:
                _log(f"  帧 {i:3d}  R=({cmd.get('right_ee.pos.x', 0):.3f},"
                     f"{cmd.get('right_ee.pos.y', 0):.3f},{cmd.get('right_ee.pos.z', 0):.3f})"
                     f"  grip={cmd.get('right_gripper.pos')}")
            time.sleep(period / max(args.speed, 1e-6))
    except KeyboardInterrupt:
        _log("中断")
    finally:
        try:
            robot.disconnect()
        except Exception:  # noqa: BLE001
            pass
    _log(f"完成。输出: {out}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
