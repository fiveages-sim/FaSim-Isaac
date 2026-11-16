#!/usr/bin/env python3
"""验证 n_action_steps 覆盖真的生效（action queue 会维持 N 帧）。

## 为什么需要
`ACTConfig` 里 `n_action_steps` 和 `chunk_size` 是有校验关系的，而
`reset()` 里 `deque(maxlen=self.config.n_action_steps)` 用的是**重置时**
的 config 值。如果先 reset 再改 config，队列 maxlen 还是旧值 —— 修了等于没修。

本脚本不碰机器人，只用假观测跑 select_action，检查：
  1) 改 config + reset 后，队列 maxlen 是否等于新值
  2) 前向次数是否约为 总调用数 / n_action_steps（而不是每帧都前向）
  3) 连续动作是否呈"每 N 帧重新跳一次"的规律

## 用法
    cd ~/lerobot_ros2 && source .venv-train/bin/activate
    python ~/FaSim-Isaac/exts/fasim.replicator.sdg/scripts/verify_action_queue.py \
        --checkpoint $HOME/act_runs/tube_pruned/checkpoints/last/pretrained_model \
        --dataset $HOME/lerobot_datasets/tube_ee
"""

from __future__ import annotations

import argparse
from pathlib import Path

import numpy as np


def main() -> int:
    ap = argparse.ArgumentParser(description="验证 n_action_steps 覆盖生效")
    ap.add_argument("--checkpoint", required=True, help="pretrained_model 目录")
    ap.add_argument("--dataset", required=True, help="LeRobot 数据集根目录")
    ap.add_argument("--episode", type=int, default=3)
    ap.add_argument("--n-action-steps", type=int, default=25)
    ap.add_argument("--frames", type=int, default=100, help="模拟多少帧闭环")
    ap.add_argument("--video-backend", default="pyav")
    args = ap.parse_args()

    import torch
    from lerobot.datasets.lerobot_dataset import LeRobotDataset
    from lerobot.policies.act.modeling_act import ACTPolicy
    from lerobot.processor.pipeline import PolicyProcessorPipeline

    ckpt = Path(args.checkpoint)
    policy = ACTPolicy.from_pretrained(str(ckpt))
    policy.to("cuda" if torch.cuda.is_available() else "cpu")
    policy.eval()
    device = next(policy.parameters()).device

    pre = post = None
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

    print("=" * 70)
    print("action queue 覆盖验证")
    print("=" * 70)
    print(f"  checkpoint n_action_steps = {policy.config.n_action_steps}")
    print(f"  chunk_size                = {policy.config.chunk_size}")

    # ---- 关键顺序：先改 config，再 reset ----
    new_n = min(args.n_action_steps, policy.config.chunk_size)
    policy.config.n_action_steps = new_n
    if policy.config.temporal_ensemble_coeff is not None:
        policy.config.temporal_ensemble_coeff = None
        print("  temporal ensemble 已关闭（与 n_action_steps>1 互斥）")
    policy.reset()

    q = getattr(policy, "_action_queue", None)
    maxlen = getattr(q, "maxlen", None)
    print(f"  reset 后队列 maxlen        = {maxlen}  "
          f"{'✅ 与应用值一致' if maxlen == new_n else '❌ 不一致！顺序错了'}")

    # ---- 用真实观测跑闭环，数前向次数 ----
    ds = LeRobotDataset(repo_id=Path(args.dataset).name, root=args.dataset,
                        video_backend=args.video_backend)
    ep_col = np.array(ds.hf_dataset["episode_index"])
    idx = np.where(ep_col == args.episode)[0]
    if len(idx) == 0:
        print(f"  ❌ 找不到 episode {args.episode}")
        return 1

    fwd_count = 0
    orig_chunk = policy.predict_action_chunk

    def counting_chunk(batch):
        nonlocal fwd_count
        fwd_count += 1
        return orig_chunk(batch)

    policy.predict_action_chunk = counting_chunk  # type: ignore[method-assign]

    acts = []
    n = args.frames
    base = int(idx[0])
    for i in range(n):
        # 用同一帧观测反复喂（模拟"观测不变"，最能暴露队列是否生效）
        sample = ds[min(base + i // 4, int(idx[-1]))]
        batch = {}
        for key, val in sample.items():
            if key in ("task", "index", "episode_index", "frame_index",
                       "timestamp", "task_index", "action"):
                continue
            t = val if isinstance(val, torch.Tensor) else torch.as_tensor(val)
            batch[key] = t.unsqueeze(0).to(device)
        if pre is not None:
            batch = pre(batch)
        with torch.no_grad():
            a = policy.select_action(batch)
        if post is not None:
            o = post({"action": a})
            a = o.get("action", a) if isinstance(o, dict) else o
        acts.append(a.squeeze(0).detach().cpu().numpy())

    a = np.stack(acts)                       # (n, 14)
    lp = a[:, 0:3]
    step = np.linalg.norm(np.diff(lp, axis=0), axis=1) * 1000
    net = float(np.linalg.norm(lp[-1] - lp[0])) * 1000

    print()
    print(f"  模拟 {n} 帧闭环：")
    print(f"    前向次数          = {fwd_count}   "
          f"（期望 ≈ {n}/{new_n} = {n/new_n:.1f}）")
    print(f"    左手净位移        = {net:.2f} mm")
    print(f"    逐帧位移 中位     = {np.median(step):.3f} mm  最大 {step.max():.2f} mm")
    # chunk 切换点会出现较大跳变
    jumps = int(np.sum(step > max(np.median(step) * 3, 1.0)))
    print(f"    跳变帧（chunk 切换）= {jumps}   （期望 ≈ {n/new_n:.0f}）")

    print()
    print("=" * 70)
    ok_fwd = fwd_count <= n / max(new_n, 1) * 1.5
    ok_len = (maxlen == new_n)
    if ok_len and ok_fwd and net > 5:
        print("  ✅ 覆盖生效：一次预测连续执行多帧，位移持续累积")
        print(f"     ⇒ 闭环不会再因稳态残差倒退（{n} 帧净走 {net:.1f}mm）")
    elif not ok_len:
        print("  ❌ 队列 maxlen 没更新 —— reset() 必须在改 config **之后**调用")
    elif not ok_fwd:
        print(f"  ❌ 前向 {fwd_count} 次，几乎每帧都重预测 —— 覆盖没生效")
    else:
        print(f"  ⚠️ 覆盖生效但位移只有 {net:.1f}mm，可能观测序列不真实（本测试复用了同一帧）")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
