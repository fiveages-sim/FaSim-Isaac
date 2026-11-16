"""实验：比较「闭环实际图像」与「训练数据集图像」，定位策略输入差异。

## 为什么
L2 离线（喂数据集图像）预测能 50 帧走 66.9mm；闭环（喂实时图像）100 帧
只要求 1mm。同一策略、同一机械臂位姿，差异只可能在**输入**。

本脚本把两边的图都存下来 + 量化比较，不靠猜：
  1. 从数据集读 episode 0 第 0 帧的图像（训练时真正看到的）
  2. 从 ROS 话题读当前实时图像（闭环时真正看到的）
  3. 逐项比较：尺寸、通道均值（BGR/RGB 互换会表现为 R 和 B 均值对调）、
     直方图相关性、像素差异
  4. 都存成 PNG，人眼可查

## 用法（Isaac 在跑、运控和相机话题正常时）
    cd ~/lerobot_ros2 && source .venv-train/bin/activate
    python ~/FaSim-Isaac/exts/fasim.replicator.sdg/scripts/compare_camera_inputs.py
"""

from __future__ import annotations

import argparse
import json
from pathlib import Path

import numpy as np

OUT_DEFAULT = Path("/tmp/sdg_cam_compare")


def _save_png(arr: np.ndarray, path: Path) -> bool:
    try:
        import cv2

        cv2.imwrite(str(path), cv2.cvtColor(arr, cv2.COLOR_RGB2BGR))
        return True
    except Exception:
        return False


def dataset_frame(ds_root: Path, episode: int = 0, frame: int = 0):
    """读数据集里某帧的相机图像（训练时模型真正看到的）。"""
    from lerobot.datasets.lerobot_dataset import LeRobotDataset

    ds = LeRobotDataset(repo_id=ds_root.name, root=str(ds_root), video_backend="pyav")
    ep = np.asarray(ds.hf_dataset["episode_index"])
    idx = np.flatnonzero(ep == episode)
    item = ds[int(idx[frame])]
    imgs = {}
    for k, v in item.items():
        if k.startswith("observation.images."):
            a = v
            if hasattr(a, "numpy"):
                a = a.numpy()
            a = np.asarray(a)
            # 数据集给的是 CHW float[0,1]
            if a.ndim == 3 and a.shape[0] in (1, 3, 4):
                a = np.transpose(a, (1, 2, 0))
            if a.dtype != np.uint8:
                a = (np.clip(a, 0, 1) * 255).astype(np.uint8)
            imgs[k.replace("observation.images.", "")] = a[..., :3]
    return imgs


def live_frames(cam_topics: dict[str, str], timeout: float = 5.0) -> dict[str, np.ndarray]:
    """从 ROS 话题读一帧图像（**原始编码**，不做任何通道转换）。"""
    import rclpy
    from rclpy.node import Node
    from rosidl_runtime_py.utilities import get_message

    rclpy.init()
    node = Node("sdg_cam_cmp")
    got: dict[str, np.ndarray] = {}
    meta: dict[str, str] = {}

    def parse(name):
        def cb(msg):
            if name in got:
                return
            h, w = int(msg.height), int(msg.width)
            enc = (msg.encoding or "").lower()
            meta[name] = f"{enc} {w}x{h} step={msg.step}"
            buf = np.frombuffer(bytes(msg.data), dtype=np.uint8)
            ch = {"rgb8": 3, "bgr8": 3, "rgba8": 4, "bgra8": 4}.get(enc, 3)
            if buf.size < h * w * ch:
                return
            img = buf[: h * w * ch].reshape(h, w, ch)
            got[name] = img[..., :3].copy()
        return cb

    subs = []
    types = dict(node.get_topic_names_and_types())
    for name, topic in cam_topics.items():
        if topic not in types:
            continue
        subs.append(node.create_subscription(
            get_message(types[topic][0]), topic, parse(name), 10))

    import time
    t0 = time.monotonic()
    while len(got) < len(cam_topics) and time.monotonic() - t0 < timeout:
        rclpy.spin_once(node, timeout_sec=0.1)
    for s in subs:
        node.destroy_subscription(s)
    node.destroy_node()
    rclpy.shutdown()
    return {k: v for k, v in got.items()}, meta


def stats(a: np.ndarray) -> dict:
    """每通道均值/标准差 —— **BGR/RGB 互换会让 R 和 B 的均值对调**，一眼可辨。"""
    return {
        "shape": list(a.shape),
        "per_channel_mean": [round(float(a[..., i].mean()), 2) for i in range(3)],
        "per_channel_std": [round(float(a[..., i].std()), 2) for i in range(3)],
        "overall_mean": round(float(a.mean()), 2),
    }


def diff_report(a: np.ndarray, b: np.ndarray, label: str) -> dict:
    """比较两张图（先 resize 到同尺寸）。"""
    import cv2

    h, w = b.shape[:2]
    if a.shape[:2] != (h, w):
        a = cv2.resize(a, (w, h), interpolation=cv2.INTER_AREA)
    d = np.abs(a.astype(np.int16) - b.astype(np.int16))
    out = {
        "label": label,
        "mean_abs_diff": round(float(d.mean()), 2),
        "max_abs_diff": int(d.max()),
        "frac_pixels_changed_gt8": round(float((d > 8).mean()), 4),
    }
    # 通道互换检验：把 a 的 R/B 换一下再比
    a_swapped = a[..., ::-1]
    d2 = np.abs(a_swapped.astype(np.int16) - b.astype(np.int16))
    out["mean_abs_diff_if_rb_swapped"] = round(float(d2.mean()), 2)
    out["rb_swap_better"] = bool(d2.mean() < d.mean() * 0.8)
    return out


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--dataset", default=str(Path.home() / "lerobot_datasets/tube_ee"))
    ap.add_argument("--episode", type=int, default=0)
    ap.add_argument("--frame", type=int, default=0)
    ap.add_argument("--out", default=str(OUT_DEFAULT))
    args = ap.parse_args()

    out = Path(args.out)
    out.mkdir(parents=True, exist_ok=True)
    report: dict = {}

    # ---- 1. 数据集图像（训练时看到的）----
    print("=" * 66)
    print("1. 从数据集读训练图像")
    print("=" * 66)
    ds_imgs = dataset_frame(Path(args.dataset).expanduser(), args.episode, args.frame)
    for k, v in ds_imgs.items():
        print(f"  {k:16s} {v.shape}  ch_mean={stats(v)['per_channel_mean']}")
        _save_png(v, out / f"dataset_{k}.png")

    # ---- 2. 实时图像（闭环时看到的，**原始编码**）----
    print()
    print("=" * 66)
    print("2. 从 ROS 话题读实时图像（原始编码，不做转换）")
    print("=" * 66)
    topics = {
        "head": "/head_camera/rgb",
        "wrist_left": "/left_hand_camera/rgb",
        "wrist_right": "/right_hand_camera/rgb",
    }
    live_imgs, live_meta = live_frames(topics)
    for k, v in live_imgs.items():
        print(f"  {k:16s} {v.shape}  ch_mean={stats(v)['per_channel_mean']}")
        print(f"  {'':16s} 原始消息: {live_meta.get(k)}")
        _save_png(v, out / f"live_{k}.png")

    # ---- 3. 比较 ----
    print()
    print("=" * 66)
    print("3. 差异比较")
    print("=" * 66)
    diffs = []
    for k in sorted(set(ds_imgs) & set(live_imgs)):
        r = diff_report(live_imgs[k], ds_imgs[k], k)
        diffs.append(r)
        print(f"  {k:16s} mean|diff|={r['mean_abs_diff']:7.2f}  "
              f"变化像素={r['frac_pixels_changed_gt8']*100:5.1f}%  "
              f"R/B互换后={r['mean_abs_diff_if_rb_swapped']:7.2f}"
              + ("   ← **R/B 互换明显更接近！**" if r["rb_swap_better"] else ""))

    report = {
        "dataset": {k: stats(v) for k, v in ds_imgs.items()},
        "live": {k: stats(v) for k, v in live_imgs.items()},
        "live_raw_meta": live_meta,
        "diffs": diffs,
    }
    (out / "report.json").write_text(
        json.dumps(report, indent=2, ensure_ascii=False), encoding="utf-8")

    # ---- 4. 结论 ----
    print()
    print("=" * 66)
    print("4. 结论")
    print("=" * 66)
    rb = [d for d in diffs if d["rb_swap_better"]]
    if rb:
        print(f"  ⚠ {len(rb)} 路相机存在 **R/B 通道互换**，"
              f"修复后差异明显减小 → 这是策略看不清的根本原因")
        print(f"    受影响: {[d['label'] for d in rb]}")
    else:
        big = [d for d in diffs if d["mean_abs_diff"] > 20]
        if big:
            print(f"  ⚠ {len(big)} 路相机差异较大（mean|diff|>20）但不是 R/B 互换："
                  f"{[d['label'] for d in big]}")
            print("    可能原因：视野/FOV 不同、分辨率缩放方式不同、或场景姿态差异")
        else:
            print("  各相机差异都很小 → 图像不是问题，应查 state / 相机内外参")
    print()
    print(f"  完整报告: {out}/report.json")
    print(f"  对比图  : {out}/dataset_*.png  与  {out}/live_*.png")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
