"""定位 head 相机的正确转换方式：4 种候选逐一试，看哪种让策略输出接近数据集。

## 背景（已由实验确认）
同一 state + 同一策略：
  数据集 head 图（640x480, 4:3） → 20 帧推进 18.21 mm  ✅
  实时 head 图（1280x720, 16:9 → 直接 resize 640x480） → 0.72 mm  ❌

## 为什么会差
Camera prim 的 aperture 是 24.95 x 13.936（**16:9**）。Isaac 的投影矩阵按
「水平 FOV 固定、垂直 FOV 随分辨率比例变化」生成：

  640x480 (4:3)  → HFOV 69.5°, VFOV 54.6°   ← 采集数据看到的
  1280x720 (16:9)→ HFOV 69.0°, VFOV 42.2°   ← ROS 话题给的

所以直接 resize(1280x720 → 640x480) 是**横向压扁**，物体变形。
正确做法要保留水平 FOV：缩放 0.5 → 640x360，再上下补边到 480。

## 本脚本
固定用数据集第 i 帧的 state，head 图分别用：
  A  dataset      —— 数据集图（基准，18.21mm）
  B  resize       —— 当前做法（直接拉伸）
  C  letterbox    —— 等比缩放 + 上下补黑边（保留水平 FOV）
  D  center_crop  —— 居中裁成 4:3 再缩放（保留垂直 FOV）
  E  pad_vert     —— 同 C，但补边用边缘复制而不是黑边
wrist 两路始终用数据集图（它们本来就是 640x480，不受影响），
这样差异**只来自 head 的转换方式**，变量隔离干净。
"""

from __future__ import annotations

import json
from pathlib import Path

import numpy as np
import torch

OUT = Path("/tmp/sdg_head_conversion")


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


def variants(live: np.ndarray, want_hw: tuple[int, int]) -> dict[str, np.ndarray]:
    """把 1280x720 的实时图转成 4 种 640x480 候选。"""
    import cv2

    H, W = want_hw
    src_h, src_w = live.shape[:2]
    out: dict[str, np.ndarray] = {}

    # B. 直接拉伸（当前做法）
    out["resize"] = cv2.resize(live, (W, H), interpolation=cv2.INTER_AREA)

    # C. letterbox：等比缩放到宽度贴合，再上下补黑边
    s = W / src_w
    nw, nh = W, int(round(src_h * s))
    scaled = cv2.resize(live, (nw, nh), interpolation=cv2.INTER_AREA)
    canvas = np.zeros((H, W, 3), dtype=np.uint8)
    top = max(0, (H - nh) // 2)
    canvas[top:top + nh] = scaled[: min(nh, H - top)]
    out["letterbox"] = canvas

    # D. center crop 成 4:3 再缩放（保留垂直 FOV，砍掉两侧）
    target_ar = W / H
    cur_ar = src_w / src_h
    if cur_ar > target_ar:                      # 太宽 → 裁两侧
        cw = int(round(src_h * target_ar))
        x0 = (src_w - cw) // 2
        cropped = live[:, x0:x0 + cw]
    else:                                       # 太高 → 裁上下
        ch = int(round(src_w / target_ar))
        y0 = (src_h - ch) // 2
        cropped = live[y0:y0 + ch, :]
    out["center_crop"] = cv2.resize(cropped, (W, H), interpolation=cv2.INTER_AREA)

    # E. 边缘复制补边（不用黑边，避免策略看到大片纯黑）
    canvas2 = np.zeros((H, W, 3), dtype=np.uint8)
    canvas2[:] = scaled[0:1, :].mean(axis=(0, 1)).astype(np.uint8)
    canvas2[top:top + nh] = scaled[: min(nh, H - top)]
    out["pad_replicate"] = canvas2
    return out


def run_once(pol, pre, post, dev, state, images, image_keys) -> np.ndarray:
    b = {"observation.state": torch.as_tensor(state, dtype=torch.float32)
         .unsqueeze(0).to(dev)}
    for key in image_keys:
        name = key.replace("observation.images.", "")
        img = images[name]
        a = np.asarray(img)
        if a.dtype != np.uint8:
            a = (np.clip(a, 0, 1) * 255).astype(np.uint8)
        t = torch.from_numpy(a[..., :3]).permute(2, 0, 1).float() / 255.0
        b[key] = t.unsqueeze(0).to(dev)
    b = pre(b)
    with torch.no_grad():
        act = pol.select_action(b)
    o = post({"action": act})
    act = o.get("action", act) if isinstance(o, dict) else act
    return np.asarray(act.squeeze(0).cpu()).reshape(-1)


def main() -> int:
    import cv2
    from pathlib import Path as P

    ds_root = P.home() / "lerobot_datasets/tube_ee"
    ck = P.home() / "act_runs/tube_pruned/checkpoints/030000/pretrained_model"
    OUT.mkdir(parents=True, exist_ok=True)

    from lerobot.datasets.lerobot_dataset import LeRobotDataset

    ds = LeRobotDataset(repo_id=ds_root.name, root=str(ds_root), video_backend="pyav")
    ep = np.asarray(ds.hf_dataset["episode_index"])
    idx = np.flatnonzero(ep == 0)
    image_keys = list(ds.meta.camera_keys)

    pol, pre, post = load_policy(ck)
    dev = next(pol.parameters()).device

    def ds_item(i):
        it = ds[int(idx[i])]
        imgs, st = {}, None
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

    # 实时 head 图（1280x720 原生）
    import rclpy
    from rclpy.node import Node
    from rosidl_runtime_py.utilities import get_message

    rclpy.init()
    node = Node("sdg_head_conv")
    box = {}
    types = dict(node.get_topic_names_and_types())
    t = "/head_camera/rgb"
    if t not in types:
        print("ERROR 找不到 /head_camera/rgb —— Isaac 没在跑或相机没发布")
        return 1

    def cb(msg):
        if box:
            return
        h, w = int(msg.height), int(msg.width)
        enc = (msg.encoding or "").lower()
        ch = {"rgb8": 3, "bgr8": 3, "rgba8": 4, "bgra8": 4}.get(enc, 3)
        buf = np.frombuffer(bytes(msg.data), dtype=np.uint8)
        if buf.size >= h * w * ch:
            box["img"] = buf[: h * w * ch].reshape(h, w, ch)[..., :3].copy()

    sub = node.create_subscription(get_message(types[t][0]), t, cb, 10)
    import time
    t0 = time.monotonic()
    while not box and time.monotonic() - t0 < 5.0:
        rclpy.spin_once(node, timeout_sec=0.1)
    node.destroy_subscription(sub)
    node.destroy_node()
    rclpy.shutdown()
    if not box:
        print("ERROR 收不到 head 图")
        return 1
    live_head = box["img"]
    print(f"实时 head: {live_head.shape}")

    ds0, _ = ds_item(0)
    ref_hw = ds0["head"].shape[:2]
    cands = variants(live_head, ref_hw)
    print(f"候选转换: {list(cands)}  目标尺寸 {ref_hw}")
    for k, v in cands.items():
        cv2.imwrite(str(OUT / f"head_{k}.png"), cv2.cvtColor(v, cv2.COLOR_RGB2BGR))
        d = float(np.abs(v.astype(np.int16) - ds0["head"].astype(np.int16)).mean())
        print(f"  {k:14s} mean|diff| vs 数据集图 = {d:6.2f}")

    # 逐候选跑序列
    N = 20
    res: dict[str, list] = {"dataset_head": []}
    for name in cands:
        res[name] = []

    for i in range(min(N, len(idx))):
        dimgs, st = ds_item(i)
        # 基准：全用数据集图
        res["dataset_head"].append(run_once(pol, pre, post, dev, st, dimgs, image_keys).tolist())
        # 变体：只换 head，wrist 用数据集图
        for name, himg in cands.items():
            mix = dict(dimgs)
            mix["head"] = himg
            res[name].append(run_once(pol, pre, post, dev, st, mix, image_keys).tolist())

    print()
    print(f"{'候选':16s} {'总位移mm':>10s} {'首帧z':>8s} {'末帧z':>8s}")
    print("-" * 46)
    for name, seq in res.items():
        A = np.array(seq)
        trav = float(np.linalg.norm(A[-1, :3] - A[0, :3])) * 1000
        print(f"{name:16s} {trav:10.2f} {A[0][2]:8.4f} {A[-1][2]:8.4f}")

    base = float(np.linalg.norm(
        np.array(res["dataset_head"])[-1, :3] - np.array(res["dataset_head"])[0, :3])) * 1000
    print()
    print("=== 结论 ===")
    best, best_v = None, -1.0
    for name, seq in res.items():
        if name == "dataset_head":
            continue
        A = np.array(seq)
        trav = float(np.linalg.norm(A[-1, :3] - A[0, :3])) * 1000
        if trav > best_v:
            best, best_v = name, trav
    print(f"  基准（数据集图）: {base:.2f} mm")
    print(f"  最佳转换: **{best}** ({best_v:.2f} mm)"
          + ("  ← 与基准接近，就是这个转换" if best_v > base * 0.6 else
             "  ← 仍远低于基准，说明不只是转换方式的问题"))

    (OUT / "report.json").write_text(
        json.dumps({"base_mm": base, "variants": {
            k: float(np.linalg.norm(np.array(v)[-1, :3] - np.array(v)[0, :3])) * 1000
            for k, v in res.items()}}, indent=2), encoding="utf-8")
    print(f"\n对比图: {OUT}/head_*.png")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
