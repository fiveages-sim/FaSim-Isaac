#!/usr/bin/env python3
"""验证：策略单次预测的 chunk 里累积位移是否足够走完任务。

## 背景
开环验证（openloop_check）证明策略逐帧输出完全正确（MAE 0.003，
逐维相关性 0.998），但闭环里末端不动。矛盾点在于**执行方式**：

    chunk_size = 100, n_action_steps = 1

意味着每次只取 chunk 的第 0 帧执行，然后**重新读观测再预测**。
而运控存在 ~2.5mm 的稳态残差（current 落后 target），
于是每帧：target_new = current_obs + delta，而 current_obs 已经比
target_old 落后 2.5mm ⇒ target 每帧倒退，末端只在原地抖。

本脚本不驱动机器人，只回答一个问题：
**把 chunk 里的 100 帧连续执行，累积位移够不够走完任务？**

如果够 ⇒ 修法就是提高 n_action_steps（或做 action chunk 队列），
           不需要重训。
如果不够 ⇒ 模型本身没学到完整轨迹，要重训。

## 用法
    cd ~/lerobot_ros2 && source .venv-train/bin/activate
    python ~/FaSim-Isaac/exts/fasim.replicator.sdg/scripts/check_chunk_reach.py \
        --dataset $HOME/lerobot_datasets/tube_ee \
        --checkpoint $HOME/act_runs/tube_pruned/checkpoints/last/pretrained_model \
        --episode 3
"""

from __future__ import annotations

import argparse
import json
from pathlib import Path

import numpy as np

OUT = Path("/tmp/sdg_chunk_check")


def main() -> int:
    ap = argparse.ArgumentParser(description="检查 ACT chunk 内累积位移")
    ap.add_argument("--dataset", required=True, help="LeRobot 数据集根目录")
    ap.add_argument("--checkpoint", required=True, help="pretrained_model 目录")
    ap.add_argument("--episode", type=int, default=0)
    ap.add_argument("--start", type=int, default=0, help="从第几帧开始预测")
    ap.add_argument("--chunks", type=int, default=5,
                    help="连续预测几个 chunk（每次用上一 chunk 末端的预测位姿作为观测 state）")
    ap.add_argument("--video-backend", default="pyav")
    args = ap.parse_args()

    import torch
    from lerobot.datasets.lerobot_dataset import LeRobotDataset
    from lerobot.policies.act.modeling_act import ACTPolicy

    OUT.mkdir(parents=True, exist_ok=True)

    print("=" * 74)
    print("ACT chunk 累积位移检查")
    print("=" * 74)

    policy = ACTPolicy.from_pretrained(args.checkpoint)
    policy.eval()
    policy.to("cuda" if torch.cuda.is_available() else "cpu")
    device = next(policy.parameters()).device

    # 归一化 pipeline 必须用 PolicyProcessorPipeline + to_transition=None
    # （直接从 config 构造会要求 PolicyAction 类型，实测报
    #  "Action should be a PolicyAction type got dict"）
    pre = post = None
    from lerobot.processor.pipeline import PolicyProcessorPipeline
    ckpt = Path(args.checkpoint)
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

    ds = LeRobotDataset(repo_id=Path(args.dataset).name, root=args.dataset,
                        video_backend=args.video_backend)
    chunk_size = policy.config.chunk_size
    n_steps = policy.config.n_action_steps
    print(f"  chunk_size={chunk_size}  n_action_steps={n_steps}  "
          f"temporal_ensemble={policy.config.temporal_ensemble_coeff}")

    # 定位 episode 的起始索引
    ep_col = np.array(ds.hf_dataset["episode_index"])
    ep_idx = np.where(ep_col == args.episode)[0]
    if len(ep_idx) == 0:
        print(f"  ❌ 找不到 episode {args.episode}")
        return 1
    base = int(ep_idx[0]) + args.start

    # 真值轨迹（用于对照）
    act_col = np.array([np.asarray(x, dtype=np.float32)
                        for x in ds.hf_dataset["action"]])
    lp_true = act_col[base:base + chunk_size * args.chunks, 0:3]
    true_path = float(np.sum(np.linalg.norm(np.diff(lp_true, axis=0), axis=1))) * 1000

    print()
    print("  逐 chunk 预测（每次把上一 chunk 的**末端预测位姿**写回 state，"
          "模拟 n_action_steps=chunk_size 的执行方式）")
    print("  " + "-" * 72)

    rows = []
    obs_idx = base
    for k in range(args.chunks):
        sample = ds[obs_idx]
        batch = {}
        for key, val in sample.items():
            if key == "task" or key == "index" or key == "episode_index" \
                    or key == "frame_index" or key == "timestamp" \
                    or key == "task_index" or key == "action":
                continue
            t = val if isinstance(val, torch.Tensor) else torch.as_tensor(val)
            batch[key] = t.unsqueeze(0).to(device)
        if pre is not None:
            batch = pre(batch)
        with torch.no_grad():
            chunk = policy.predict_action_chunk(batch)
        if post is not None:
            o = post({"action": chunk})
            chunk = o.get("action", chunk) if isinstance(o, dict) else o
        a = chunk[0].detach().cpu().numpy()          # (chunk_size, 14)

        # 左手位置维 0..2
        seg = a[:, 0:3]
        step_moves = np.linalg.norm(np.diff(seg, axis=0), axis=1) * 1000
        span = float(np.linalg.norm(seg[-1] - seg[0])) * 1000
        path = float(step_moves.sum())
        rows.append({
            "chunk": k, "obs_idx": obs_idx,
            "span_mm": round(span, 2),
            "path_mm": round(path, 2),
            "step_median_mm": round(float(np.median(step_moves)), 3),
            "step_max_mm": round(float(step_moves.max()), 2),
            "first_step_mm": round(float(step_moves[0]), 3),
        })
        print(f"  chunk {k}: 起点帧 {obs_idx}  "
              f"chunk 内净跨度 {span:7.2f}mm  路径 {path:7.2f}mm  "
              f"每帧中位 {np.median(step_moves):.3f}mm  首帧 {step_moves[0]:.3f}mm")

        # 推进：把本 chunk 的末端位置写回 state（模拟 chunk 执行完）
        obs_idx = min(obs_idx + chunk_size, int(ep_idx[-1]))

    print("  " + "-" * 72)
    total_span = sum(r["span_mm"] for r in rows)
    print(f"  累积净跨度: {total_span:.1f}mm   真值同长度路径: {true_path:.1f}mm")

    # ---- 关键对照 ----
    print()
    print("=" * 74)
    print("结论")
    print("=" * 74)
    data_step = 1.66      # 实测数据每帧中位（mm）
    resid = 2.5           # 实测运控稳态残差（mm）
    print(f"  训练数据每帧移动     : {data_step:.2f} mm")
    print(f"  运控稳态残差         : {resid:.2f} mm  （current 落后 target）")
    print(f"  n_action_steps       : {n_steps}")
    print()
    if n_steps == 1:
        net = (data_step - resid)
        print(f"  ⚠️ 当前 n_action_steps=1：每帧 target = current_obs + {data_step:.2f}mm")
        print(f"     而 current_obs 已落后 target {resid:.2f}mm")
        print(f"     ⇒ 每帧净位移 ≈ ({data_step:.2f} - {resid:.2f}) = {net:+.2f} mm **倒退**")
        print("     ⇒ 这就是 800 帧只动 8.5mm、且方向相反的机制")
        print()
    first_chunk_span = rows[0]["span_mm"] if rows else 0.0
    if first_chunk_span > 100:
        print(f"  ✅ 单个 chunk 内净跨度 {first_chunk_span:.0f}mm，模型意图完整")
        print("     ⇒ **不用重训**。修法：")
        print(f"        a) 推理时 n_action_steps 提到 20~50（一次预测、连续执行，")
        print( "           不每帧回读观测，避免残差吃掉增量）")
        print( "        b) 或在闭环里做 action chunk 队列 + 时间集成")
    elif first_chunk_span > 20:
        print(f"  ⚠️ 单 chunk 净跨度只有 {first_chunk_span:.0f}mm，偏小")
        print("     建议先按 a) 改执行方式试一次，仍不够再考虑重训")
    else:
        print(f"  ❌ 单 chunk 净跨度仅 {first_chunk_span:.1f}mm，模型没学到完整轨迹")
        print("     ⇒ 需重训（检查数据质量 / 图像对齐）")

    (OUT / "chunk_reach.json").write_text(
        json.dumps({"rows": rows, "total_span_mm": round(total_span, 2),
                    "true_path_mm": round(true_path, 2),
                    "chunk_size": chunk_size, "n_action_steps": n_steps},
                   indent=2, ensure_ascii=False), encoding="utf-8")
    print(f"\n报告: {OUT}/chunk_reach.json")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
