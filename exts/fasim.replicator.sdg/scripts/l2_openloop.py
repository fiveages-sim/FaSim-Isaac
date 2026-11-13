#!/usr/bin/env python3
"""L2 开环推理 —— 用训练好的 ACT 在已有 episode 上逐帧预测 action，与真值对比。

## 这是哪一层评测
L1 离线（训练 loss）只说明拟合得好，**不能说明策略能用**。
L2 开环把策略输出与真值逐帧对比，能在**不闭环、不仿真**的前提下发现：
  - 动作量级错（比如全部饱和到 ±1）
  - 动作抖动（相邻帧跳变）
  - 某几维完全不动（学漏了）

真实成功率要靠 L3 闭环，但 L2 能先排掉大部分低级错误。

## 重要限制（必须知道）
ACT 是多模态策略：**必须同时给 observation.state 和 observation.images.***。
本脚本的初版只喂 state 会失败。完整实现需要从 LeRobot 数据集的视频里解码
对应帧 —— 这里用 `LeRobotDataset` 的 `__getitem__` 更可靠（它内部处理
视频解码与时间戳对齐）。

## 用法
```bash
cd ~/lerobot_ros2 && source .venv-train/bin/activate
python ~/FaSim-Isaac/exts/fasim.replicator.sdg/scripts/l2_openloop.py \
    --dataset ~/lerobot_datasets/tube_25eps \
    --checkpoint ~/act_runs/tube25/checkpoints/last/pretrained_model \
    --episode 0
```
"""

from __future__ import annotations

import argparse
import json
from pathlib import Path

import numpy as np


def _open_dataset(root: Path, episode: int, video_backend: str = "pyav"):
    """用官方 LeRobotDataset 打开（自动处理视频解码与归一化 stats）。"""
    from lerobot.datasets.lerobot_dataset import LeRobotDataset

    ds = LeRobotDataset(
        repo_id=root.name,
        root=str(root),
        video_backend=video_backend,
    )
    # 该 episode 的帧索引
    ep_idx = np.asarray(ds.hf_dataset["episode_index"])
    idx = np.flatnonzero(ep_idx == episode)
    return ds, idx


def _load_policy(ckpt: Path):
    """加载 ACTPolicy 与它的 pre/post processor（归一化）。

    **为什么必须带 processor**（lerobot 0.5.x）：
    ``policy.select_action(batch)`` 只做**模型前向**，既不做输入归一化，
    也不做输出反归一化。归一化逻辑被抽到 ``PolicyProcessorPipeline`` 里，
    训练时自动保存到 checkpoint：

        policy_preprocessor.json  + ..._normalizer_processor.safetensors
        policy_postprocessor.json + ..._unnormalizer_processor.safetensors

    所以推理必须是：
        raw_obs -> preprocessor(归一化) -> select_action -> postprocessor(反归一化) -> action

    漏掉这一步的典型症状：预测值量级是真值的上千倍、大量饱和到 ±1
    （因为模型输出的是归一化空间的值，被当成了弧度直接比较）。
    """
    from lerobot.policies.act.modeling_act import ACTPolicy

    policy = ACTPolicy.from_pretrained(str(ckpt))
    policy.eval()

    pre = post = None
    try:
        from lerobot.processor.pipeline import PolicyProcessorPipeline

        for fname, tag in (("policy_preprocessor.json", "pre"),
                           ("policy_postprocessor.json", "post")):
            if (ckpt / fname).is_file():
                pipe = PolicyProcessorPipeline.from_pretrained(
                    pretrained_model_name_or_path=str(ckpt),
                    config_filename=fname,
                    to_transition=None,
                )
                if tag == "pre":
                    pre = pipe
                else:
                    post = pipe
    except Exception as exc:  # noqa: BLE001
        print(f"[l2] WARN processor 加载失败（预测值将不可比）: {exc}")
    return policy, pre, post


def main() -> int:
    ap = argparse.ArgumentParser(description="ACT 开环推理评测（L2）")
    ap.add_argument("--dataset", required=True, help="LeRobot 数据集根目录")
    ap.add_argument("--checkpoint", required=True, help="pretrained_model 目录")
    ap.add_argument("--episode", type=int, default=0)
    ap.add_argument("--out", default="", help="输出目录")
    ap.add_argument("--max-frames", type=int, default=0, help="只跑前 N 帧")
    ap.add_argument("--start", type=int, default=0,
                    help="从第几帧开始（默认 0）。注意开头常有静止段，"
                         "测动作质量建议给个几百跳过静止")
    ap.add_argument("--video-backend", default="pyav",
                    help="视频后端（torchcodec 在部分机器上缺 libavutil）")
    args = ap.parse_args()

    root = Path(args.dataset).expanduser().resolve()
    ckpt = Path(args.checkpoint).expanduser().resolve()
    out = (Path(args.out).expanduser() if args.out
           else ckpt.parent.parent / "l2")
    out.mkdir(parents=True, exist_ok=True)

    ds, idx = _open_dataset(root, args.episode, args.video_backend)
    if idx.size == 0:
        raise SystemExit(f"episode {args.episode} 不在 {root}")
    if args.start > 0:
        idx = idx[args.start:]
    if args.max_frames > 0:
        idx = idx[: args.max_frames]
    print(f"[l2] episode {args.episode}: {len(idx)} 帧  from {root.name}")

    policy, pre, post = _load_policy(ckpt)
    import torch

    dev = next(policy.parameters()).device
    print(f"[l2] device: {dev}  preprocessor={'有' if pre else '无'} "
          f"postprocessor={'有' if post else '无'}")
    if pre is None:
        print("[l2] WARN 没有 preprocessor —— 输入未归一化，结果不可信")

    preds, gts = [], []
    with torch.no_grad():
        for k, i in enumerate(idx):
            item = ds[int(i)]
            batch = {}
            for key, val in item.items():
                if key == "task":
                    continue
                t = val if isinstance(val, torch.Tensor) else torch.as_tensor(val)
                batch[key] = t.unsqueeze(0).to(dev)

            # 1) 归一化输入
            if pre is not None:
                batch = pre(batch)
            # 2) 模型前向
            a = policy.select_action(batch)
            # 3) 反归一化输出（postprocessor 返回 dict，取 action 键）
            if post is not None:
                out = post({"action": a})
                if isinstance(out, dict):
                    a = out.get("action", a)
                else:
                    a = out
            arr = np.asarray(a.squeeze(0).detach().cpu()).reshape(-1)
            preds.append(arr.astype(np.float32))
            gts.append(np.asarray(item["action"]).reshape(-1).astype(np.float32))
            if (k + 1) % 100 == 0:
                print(f"[l2]   {k+1}/{len(idx)}")

    pred = np.array(preds, dtype=np.float32)
    gt = np.array(gts, dtype=np.float32)

    mae_dim = np.abs(pred - gt).mean(axis=0)
    mae = float(np.abs(pred - gt).mean())
    print(f"\n[l2] 整体 MAE: {mae:.6f}")
    print(f"[l2] 量级: 真值 mean|a|={np.abs(gt).mean():.6f}  "
          f"预测 mean|a|={np.abs(pred).mean():.6f}")

    corrs = []
    for d in range(pred.shape[1]):
        a, b = pred[:, d], gt[:, d]
        corrs.append(np.nan if (a.std() < 1e-9 or b.std() < 1e-9)
                     else float(np.corrcoef(a, b)[0, 1]))
    corrs = np.array(corrs)
    valid = ~np.isnan(corrs)
    print(f"[l2] 逐维相关性: mean={np.nanmean(corrs):.3f} "
          f"中位={np.nanmedian(corrs):.3f} "
          f">0.8 的维度 {int((corrs[valid]>0.8).sum())}/{valid.sum()}")

    jp = np.linalg.norm(np.diff(pred, axis=0), axis=1) if len(pred) > 1 else np.array([0.0])
    jg = np.linalg.norm(np.diff(gt, axis=0), axis=1) if len(gt) > 1 else np.array([0.0])
    ratio = float(np.median(jp) / max(np.median(jg), 1e-12))
    print(f"[l2] 相邻帧变化: 真值 med={np.median(jg):.6f} 预测 med={np.median(jp):.6f} "
          f"({ratio:.2f}x)")

    sat = float((np.abs(pred) > 0.99).mean())
    dead = int((np.abs(pred).std(axis=0) < 1e-6).sum())

    issues = []
    if mae > 0.1:
        issues.append(f"MAE 偏大 ({mae:.4f})")
    if np.nanmean(corrs) < 0.5:
        issues.append(f"相关性低 ({np.nanmean(corrs):.2f})")
    if ratio > 3:
        issues.append(f"抖动 {ratio:.1f}x")
    if sat > 0.01:
        issues.append(f"{sat*100:.1f}% 饱和")
    if dead:
        issues.append(f"{dead} 维不变")

    print("\n[l2] 判定: " + ("✅ 无明显问题" if not issues else "⚠️ " + "; ".join(issues)))

    res = {
        "episode": args.episode, "frames": len(pred), "mae": mae,
        "pred_abs_mean": float(np.abs(pred).mean()),
        "gt_abs_mean": float(np.abs(gt).mean()),
        "corr_mean": float(np.nanmean(corrs)),
        "corr_median": float(np.nanmedian(corrs)),
        "jitter_ratio": ratio,
        "saturated_frac": sat,
        "dead_dims": dead,
        "issues": issues,
        "per_dim_mae": mae_dim.tolist(),
        "per_dim_corr": corrs.tolist(),
    }
    p = out / f"l2_ep{args.episode}.json"
    p.write_text(json.dumps(res, indent=2, ensure_ascii=False), encoding="utf-8")
    print(f"\n[l2] 结果: {p}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
