"""逐相机消融 + 拼图对比：定位到底是哪一路相机让策略"看不清"。

## 已知
  三路全用数据集图 → 20 帧推进 18.21 mm
  三路全用实时图   →  0.72 mm
  只换 head       → 13.80 mm（直接 resize）

所以**腕部**的影响比 head 更大？需要逐路消融确认。

## 本脚本
1. 逐相机消融：基准全数据集图，然后单独把 head / wrist_left / wrist_right
   换成实时图（resize 到数据集尺寸），看各自的位移损失
2. 组合：三路全实时
3. 出一张拼图 PNG：每路相机一行，列出 dataset / live / 各转换方式，
   方便人眼确认"是不是真的同一画面"

## 用法（Isaac 在跑）
    cd ~/lerobot_ros2 && source .venv-train/bin/activate
    python ~/FaSim-Isaac/exts/fasim.replicator.sdg/scripts/ablate_cameras.py
"""

from __future__ import annotations

import json
from pathlib import Path

import numpy as np
import torch

OUT = Path("/tmp/sdg_cam_ablation")
TOPICS = {"head": "/head_camera/rgb",
          "wrist_left": "/left_hand_camera/rgb",
          "wrist_right": "/right_hand_camera/rgb"}


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


def grab_live(timeout: float = 6.0) -> dict[str, np.ndarray]:
    import time

    import rclpy
    from rclpy.node import Node
    from rosidl_runtime_py.utilities import get_message

    rclpy.init()
    node = Node("sdg_cam_ablate")
    got: dict[str, np.ndarray] = {}
    meta: dict[str, str] = {}
    types = dict(node.get_topic_names_and_types())

    def mk(name):
        def cb(msg):
            if name in got:
                return
            h, w = int(msg.height), int(msg.width)
            enc = (msg.encoding or "").lower()
            ch = {"rgb8": 3, "bgr8": 3, "rgba8": 4, "bgra8": 4}.get(enc, 3)
            buf = np.frombuffer(bytes(msg.data), dtype=np.uint8)
            if buf.size >= h * w * ch:
                got[name] = buf[: h * w * ch].reshape(h, w, ch)[..., :3].copy()
                meta[name] = f"{enc} {w}x{h}"
        return cb

    subs = [node.create_subscription(get_message(types[t][0]), t, mk(n), 10)
            for n, t in TOPICS.items() if t in types]
    t0 = time.monotonic()
    while len(got) < len(TOPICS) and time.monotonic() - t0 < timeout:
        rclpy.spin_once(node, timeout_sec=0.1)
    for s in subs:
        node.destroy_subscription(s)
    node.destroy_node()
    rclpy.shutdown()
    for k, v in meta.items():
        print(f"  live {k:12s} {v}")
    return got


def contact_sheet(rows: list[tuple[str, list[tuple[str, np.ndarray]]]], path: Path) -> bool:
    """把每路相机的多张候选图拼成一张，带标题。

    **每格统一成固定尺寸**（tile_h × tile_w）再拼 —— 不同相机/不同来源的
    长宽比不同（head 是 16:9，腕部 4:3），直接 hstack 会因高度不一报错。
    """
    try:
        import cv2
    except Exception:
        return False

    TILE_W, TILE_H = 320, 240          # 4:3 画布，内容 letterbox 居中
    PAD_TOP = 22
    tiles: list[np.ndarray] = []

    def make_tile(name: str, img: np.ndarray) -> np.ndarray:
        src = np.asarray(img)
        if src.ndim == 2:
            src = np.stack([src] * 3, axis=-1)
        src = src[..., :3].astype(np.uint8)
        h, w = src.shape[:2]
        if h <= 0 or w <= 0:
            src, h, w = np.zeros((TILE_H, TILE_W, 3), np.uint8), TILE_H, TILE_W
        s = min(TILE_W / w, TILE_H / h)
        nw, nh = max(1, int(round(w * s))), max(1, int(round(h * s)))
        r = cv2.resize(src, (nw, nh), interpolation=cv2.INTER_AREA)
        canvas = np.full((TILE_H, TILE_W, 3), 18, dtype=np.uint8)
        y0, x0 = (TILE_H - nh) // 2, (TILE_W - nw) // 2
        canvas[y0:y0 + nh, x0:x0 + nw] = r
        out = cv2.copyMakeBorder(canvas, PAD_TOP, 4, 2, 2,
                                 cv2.BORDER_CONSTANT, value=(30, 30, 30))
        cv2.putText(out, name[:28], (4, 15), cv2.FONT_HERSHEY_SIMPLEX,
                    0.42, (255, 255, 255), 1, cv2.LINE_AA)
        return out

    max_cols = max(len(cols) for _, cols in rows)
    for label, cols in rows:
        line = [make_tile(n, im) for n, im in cols]
        while len(line) < max_cols:
            line.append(np.full((TILE_H + PAD_TOP + 4, TILE_W + 4, 3), 18, np.uint8))
        strip = np.hstack(line)
        strip = cv2.copyMakeBorder(strip, 20, 2, 2, 2,
                                   cv2.BORDER_CONSTANT, value=(0, 0, 0))
        cv2.putText(strip, label, (6, 15), cv2.FONT_HERSHEY_SIMPLEX,
                    0.5, (0, 255, 255), 1, cv2.LINE_AA)
        tiles.append(strip)
    cv2.imwrite(str(path), cv2.cvtColor(np.vstack(tiles), cv2.COLOR_RGB2BGR))
    return True


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
    keys = list(ds.meta.camera_keys)

    pol, pre, post = load_policy(ck)
    dev = next(pol.parameters()).device

    def ds_item(i):
        it = ds[int(idx[i])]
        imgs = {}
        for k in keys:
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

    print("读取实时图像…")
    live = grab_live()

    ds0, _ = ds_item(0)
    # 实时图统一 resize 到对应数据集尺寸（保持与推理流程一致）
    live_rs: dict[str, np.ndarray] = {}
    for name, img in live.items():
        ref = ds0.get(name)
        if ref is None:
            continue
        live_rs[name] = cv2.resize(img, (ref.shape[1], ref.shape[0]),
                                   interpolation=cv2.INTER_AREA)

    # ---- 拼图（人眼确认是否同一画面）----
    rows = []
    for name in ("head", "wrist_left", "wrist_right"):
        cols = []
        if name in ds0:
            cols.append(("dataset " + str(ds0[name].shape[:2]), ds0[name]))
        if name in live:
            cols.append(("live " + str(live[name].shape[:2]), live[name]))
        if name in live_rs:
            cols.append(("live->resize", live_rs[name]))
        if name in ds0 and name in live_rs:
            cols.append(("|diff|", np.abs(
                live_rs[name].astype(np.int16) - ds0[name].astype(np.int16)
            ).clip(0, 255).astype(np.uint8) * 2))
        rows.append((name, cols))
    sheet = OUT / "cameras.png"
    try:
        ok = contact_sheet(rows, sheet)
        print(f"拼图: {sheet}" if ok else "拼图生成失败（不影响消融）")
    except Exception as exc:  # noqa: BLE001
        print(f"拼图失败（不影响消融）: {exc}")

    # ---- 逐相机消融 ----
    def run_seq(swap: set[str]) -> float:
        res = []
        for i in range(min(20, len(idx))):
            dimgs, st = ds_item(i)
            imgs = dict(dimgs)
            for name in swap:
                if name in live_rs:
                    imgs[name] = live_rs[name]
            b = {"observation.state": torch.as_tensor(st, dtype=torch.float32)
                 .unsqueeze(0).to(dev)}
            for key in keys:
                nm = key.replace("observation.images.", "")
                a = np.asarray(imgs[nm], dtype=np.uint8)
                t = torch.from_numpy(a[..., :3]).permute(2, 0, 1).float() / 255.0
                b[key] = t.unsqueeze(0).to(dev)
            b = pre(b)
            with torch.no_grad():
                act = pol.select_action(b)
            o = post({"action": act})
            act = o.get("action", act) if isinstance(o, dict) else act
            res.append(np.asarray(act.squeeze(0).cpu()).reshape(-1))
        A = np.array(res)
        return float(np.linalg.norm(A[-1, :3] - A[0, :3])) * 1000

    print()
    print("=== 逐相机消融（替换成实时图，其余用数据集图）===")
    cases = {
        "基线（全数据集图）": set(),
        "只换 head": {"head"},
        "只换 wrist_left": {"wrist_left"},
        "只换 wrist_right": {"wrist_right"},
        "两个腕部都换": {"wrist_left", "wrist_right"},
        "三路全换": {"head", "wrist_left", "wrist_right"},
    }
    out = {}
    for label, swap in cases.items():
        mm = run_seq(swap)
        out[label] = round(mm, 2)
        print(f"  {label:20s} {mm:8.2f} mm")

    base = out["基线（全数据集图）"]
    print()
    print("=== 归因（位移损失 = 基线 - 该组）===")
    for label in ("只换 head", "只换 wrist_left", "只换 wrist_right"):
        loss = base - out[label]
        print(f"  {label:20s} 损失 {loss:6.2f} mm")

    (OUT / "report.json").write_text(
        json.dumps({"displacements_mm": out, "live_meta": {
            k: list(v.shape) for k, v in live.items()}}, indent=2), encoding="utf-8")
    print()
    print(f"报告: {OUT}/report.json")
    print(f"拼图: {sheet}   ← 先看这个，确认是不是同一画面")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
