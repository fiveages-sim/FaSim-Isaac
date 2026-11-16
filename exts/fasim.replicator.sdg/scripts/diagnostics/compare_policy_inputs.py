"""决定性实验：同一时刻，分别用「数据集图像」和「实时图像」喂策略，比输出。

## 目的
闭环里策略输出极小（100 帧只要求 1mm），而 L2 里同一模型输出正常
（50 帧 66.9mm）。差异只在输入。本脚本隔离变量：

  输入 = 同一个 observation.state（都用数据集第 0 帧的 32 维关节角）
  变量 = 图像来源（数据集 / 实时话题）

三次推理：
  A. 图像用**数据集**第 0 帧  → 这是训练分布内的输入（应输出正常动作）
  B. 图像用**实时**话题（resize 到 640x480，与推理流程一致）
  C. 图像用实时，但**先做 R/B 互换**（验证通道假设是否成立）

如果 A 正常、B 静止 ⇒ 图像是问题（FOV/构图差异）。
如果 A 和 B 都静止 ⇒ 不是图像问题，是 state 或模型本身。

## 用法（Isaac 在跑时）
    cd ~/lerobot_ros2 && source .venv-train/bin/activate
    python ~/FaSim-Isaac/exts/fasim.replicator.sdg/scripts/compare_policy_inputs.py
"""

from __future__ import annotations

import argparse
import json
from pathlib import Path

import numpy as np
import torch

OUT = Path("/tmp/sdg_policy_input_cmp2")


def load_policy(ck: Path):
    from lerobot.policies.act.modeling_act import ACTPolicy
    from lerobot.processor.pipeline import PolicyProcessorPipeline

    pol = ACTPolicy.from_pretrained(str(ck))
    pol.eval()
    pre = PolicyProcessorPipeline.from_pretrained(
        pretrained_model_name_or_path=str(ck),
        config_filename="policy_preprocessor.json", to_transition=None)
    post = PolicyProcessorPipeline.from_pretrained(
        pretrained_model_name_or_path=str(ck),
        config_filename="policy_postprocessor.json", to_transition=None)
    return pol, pre, post


def run_once(pol, pre, post, dev, state, images: dict[str, np.ndarray],
             image_keys: list[str]) -> np.ndarray:
    """用给定 state + images 做一次推理，返回 14 维动作。"""
    b: dict = {"observation.state": torch.as_tensor(
        state, dtype=torch.float32).unsqueeze(0).to(dev)}
    for key in image_keys:
        name = key.replace("observation.images.", "")
        img = images.get(name)
        if img is None:
            continue
        a = np.asarray(img)
        if a.dtype != np.uint8:
            a = (np.clip(a, 0, 1) * 255).astype(np.uint8)
        t = torch.from_numpy(a[..., :3]).permute(2, 0, 1).float() / 255.0
        b[key] = t.unsqueeze(0).to(dev)
    b = pre(b)
    with torch.no_grad():
        act = pol.select_action(b)
    o = post({"action": act})
    act = o.get("action", act) if isinstance(o, dict) else o
    return np.asarray(act.squeeze(0).cpu()).reshape(-1)


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--dataset", default=str(Path.home() / "lerobot_datasets/tube_grip"))
    ap.add_argument("--checkpoint",
                    default=str(Path.home() / "act_runs/tube_grip_30k/checkpoints/030000/pretrained_model"))
    ap.add_argument("--episode", type=int, default=0)
    ap.add_argument("--frames", type=int, default=20,
                    help="用数据集前 N 帧的 state 序列，让策略有机会推进")
    args = ap.parse_args()
    OUT.mkdir(parents=True, exist_ok=True)

    from lerobot.datasets.lerobot_dataset import LeRobotDataset

    root = Path(args.dataset).expanduser()
    ds = LeRobotDataset(repo_id=root.name, root=str(root), video_backend="pyav")
    ep = np.asarray(ds.hf_dataset["episode_index"])
    idx = np.flatnonzero(ep == args.episode)

    pol, pre, post = load_policy(Path(args.checkpoint).expanduser())
    dev = next(pol.parameters()).device
    image_keys = [k for k in ds.meta.camera_keys]
    print(f"image_keys: {image_keys}")

    # ---- 取数据集前 N 帧（图像 + state）----
    def ds_item(i):
        it = ds[int(idx[i])]
        imgs = {}
        for k in image_keys:
            a = it[k]
            a = a.numpy() if hasattr(a, "numpy") else np.asarray(a)
            if a.ndim == 3 and a.shape[0] in (1, 3, 4):
                a = np.transpose(a, (1, 2, 0))
            if a.dtype != np.uint8:
                a = (np.clip(a, 0, 1) * 255).astype(np.uint8)
            imgs[k.replace("observation.images.", "")] = a[..., :3]
        st = it["observation.state"]
        st = st.numpy() if hasattr(st, "numpy") else np.asarray(st)
        return imgs, np.asarray(st, dtype=np.float32).reshape(-1)

    # ---- 取实时图像（一帧，复用）----
    import rclpy
    from rclpy.node import Node
    from rosidl_runtime_py.utilities import get_message

    rclpy.init()
    node = Node("sdg_pol_cmp")
    topics = {"head": "/head_camera/rgb", "wrist_left": "/left_hand_camera/rgb",
              "wrist_right": "/right_hand_camera/rgb"}
    live: dict[str, np.ndarray] = {}
    types = dict(node.get_topic_names_and_types())

    def mk(name):
        def cb(msg):
            if name in live:
                return
            h, w = int(msg.height), int(msg.width)
            enc = (msg.encoding or "").lower()
            ch = {"rgb8": 3, "bgr8": 3, "rgba8": 4, "bgra8": 4}.get(enc, 3)
            buf = np.frombuffer(bytes(msg.data), dtype=np.uint8)
            if buf.size >= h * w * ch:
                live[name] = buf[: h * w * ch].reshape(h, w, ch)[..., :3].copy()
        return cb

    subs = [node.create_subscription(get_message(types[t][0]), t, mk(n), 10)
            for n, t in topics.items() if t in types]
    import time
    t0 = time.monotonic()
    while len(live) < len(topics) and time.monotonic() - t0 < 5.0:
        rclpy.spin_once(node, timeout_sec=0.1)
    for s in subs:
        node.destroy_subscription(s)
    node.destroy_node()
    rclpy.shutdown()
    print(f"实时图像: {[(k, v.shape) for k, v in live.items()]}")

    # 实时图 resize 到数据集尺寸（模拟推理流程）
    import cv2
    live_rs = {}
    ds0_imgs, _ = ds_item(0)
    for name, img in live.items():
        ref = ds0_imgs.get(name)
        if ref is not None:
            live_rs[name] = cv2.resize(img, (ref.shape[1], ref.shape[0]),
                                       interpolation=cv2.INTER_AREA)
        else:
            live_rs[name] = img

    results = {"A_dataset_imgs": [], "B_live_imgs": [],
               "C_live_rb_swapped": []}

    print()
    print(f"{'帧':>3s} {'A数据集图':>26s} {'B实时图':>26s}")
    for i in range(min(args.frames, len(idx))):
        dimgs, st = ds_item(i)
        a = run_once(pol, pre, post, dev, st, dimgs, image_keys)
        b = run_once(pol, pre, post, dev, st, live_rs, image_keys)
        results["A_dataset_imgs"].append(a.tolist())
        results["B_live_imgs"].append(b.tolist())
        if i == 0:
            swapped = {k: v[..., ::-1].copy() for k, v in live_rs.items()}
            c = run_once(pol, pre, post, dev, st, swapped, image_keys)
            results["C_live_rb_swapped"].append(c.tolist())
        if i % 5 == 0 or i == min(args.frames, len(idx)) - 1:
            print(f"{i:3d}  [{a[0]:6.3f},{a[1]:6.3f},{a[2]:6.3f}]  "
                  f"[{b[0]:6.3f},{b[1]:6.3f},{b[2]:6.3f}]")

    A = np.array(results["A_dataset_imgs"])
    B = np.array(results["B_live_imgs"])
    travA = float(np.linalg.norm(A[-1, :3] - A[0, :3])) * 1000
    travB = float(np.linalg.norm(B[-1, :3] - B[0, :3])) * 1000
    print()
    print(f"A（数据集图像）总位移: {travA:.2f} mm")
    print(f"B（实时图像）  总位移: {travB:.2f} mm")
    if "C_live_rb_swapped" in results and results["C_live_rb_swapped"]:
        C = np.array(results["C_live_rb_swapped"])[0]
        print(f"C（实时图 R/B 互换）首帧动作: "
              f"[{C[0]:.3f}, {C[1]:.3f}, {C[2]:.3f}]")
        print(f"A 首帧动作:                    "
              f"[{A[0][0]:.3f}, {A[0][1]:.3f}, {A[0][2]:.3f}]")
        print(f"B 首帧动作:                    "
              f"[{B[0][0]:.3f}, {B[0][1]:.3f}, {B[0][2]:.3f}]")
    print()
    print("=== 结论 ===")
    if travA > 5 * max(travB, 0.1):
        print(f"  ⚠ 用数据集图像能正常推进（{travA:.1f}mm），"
              f"用实时图像几乎不动（{travB:.1f}mm）")
        print("  ⇒ **图像是根因**。下一步查 FOV/内参/构图差异。")
    else:
        print(f"  两种图像输出相近（{travA:.1f} vs {travB:.1f}mm）"
              f" ⇒ 图像不是主要问题，需查 state 或模型。")

    (OUT / "report.json").write_text(
        json.dumps(results, indent=2), encoding="utf-8")
    print(f"\n结果: {OUT}/report.json")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
