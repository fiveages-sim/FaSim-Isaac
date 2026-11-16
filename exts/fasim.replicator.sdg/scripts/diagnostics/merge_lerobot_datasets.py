"""把多个单-episode 的 LeRobot 数据集合并成一个多-episode 数据集。

背景
====
采集时如果逐包导出（或像早期迁移那样），会得到
``~/lerobot_datasets/run_042/``、``run_043/`` … 每个都是**独立数据集**。
训练时需要的是一个数据集、多条 episode，所以最后要合并。

为什么不用「重新从 frames 批量导出」
----------------------------------
早期迁移已经把大部分包的 frames 裁剪成 8 帧（保留用于质监），
**原始图像只剩在各自的数据集里**。所以只能走「合并已有数据集」这条路。

LeRobot v3.4 自带 ``merge_datasets(datasets, output_repo_id, output_dir)``，
本脚本只是把它包成命令行，并加上包名筛选/校验。

用法
====
::

    # 合并 run_042..run_066 成一个数据集
    python merge_lerobot_datasets.py \
        --pattern 'run_04[2-9]' --pattern 'run_05*' --pattern 'run_06*' \
        --out g1d_tube_v2

    # 先看会合并哪些（不真的跑）
    python merge_lerobot_datasets.py --pattern 'run_*' --out xxx --dry-run
"""

from __future__ import annotations

import argparse
import fnmatch
import json
import os
import shutil
import sys
from pathlib import Path


def _log(msg: str) -> None:
    print(f"[merge] {msg}", flush=True)


def find_datasets(root: Path, patterns: list[str], exclude: list[str]) -> list[Path]:
    """在 root 下找符合 pattern 的子目录，且是有效数据集（有 meta/info.json）。"""
    found: list[Path] = []
    for p in sorted(root.iterdir()):
        if not p.is_dir():
            continue
        if patterns and not any(fnmatch.fnmatch(p.name, pat) for pat in patterns):
            continue
        if any(fnmatch.fnmatch(p.name, pat) for pat in exclude):
            continue
        info = p / "meta" / "info.json"
        if not info.is_file():
            continue
        try:
            meta = json.loads(info.read_text(encoding="utf-8"))
        except Exception:  # noqa: BLE001
            _log(f"  WARN 跳过（info.json 读不了）: {p.name}")
            continue
        n_ep = int(meta.get("total_episodes", 0) or 0)
        n_fr = int(meta.get("total_frames", 0) or 0)
        if n_ep == 0 or n_fr == 0:
            _log(f"  WARN 跳过（空数据集 {n_ep}ep/{n_fr}帧）: {p.name}")
            continue
        found.append(p)
    return found


def main() -> int:
    ap = argparse.ArgumentParser(
        description="合并多个 LeRobot 数据集成一个多-episode 数据集")
    ap.add_argument("--root", default="", help="数据集根目录（默认 ~/lerobot_datasets）")
    ap.add_argument("--pattern", action="append", default=[],
                    help="要合并的子目录名 glob（可多次给，默认 run_*）")
    ap.add_argument("--exclude", action="append", default=[],
                    help="排除的子目录名 glob（可多次给）")
    ap.add_argument("--out", required=True, help="输出数据集名（放在 --root 下）")
    ap.add_argument("--overwrite", action="store_true", help="输出已存在时先删除")
    ap.add_argument("--dry-run", action="store_true", help="只列出会合并哪些")
    args = ap.parse_args()

    root = Path(args.root).expanduser() if args.root else Path.home() / "lerobot_datasets"
    if not root.is_dir():
        _log(f"ERROR 数据集根目录不存在: {root}")
        return 1

    patterns = args.pattern or ["run_*"]
    sessions = find_datasets(root, patterns, args.exclude)
    if not sessions:
        _log(f"ERROR {root} 下没找到匹配的数据集（pattern={patterns}）")
        return 1

    _log(f"找到 {len(sessions)} 个数据集：")
    total_ep = total_fr = 0
    for s in sessions:
        meta = json.loads((s / "meta" / "info.json").read_text(encoding="utf-8"))
        ep = int(meta.get("total_episodes", 0) or 0)
        fr = int(meta.get("total_frames", 0) or 0)
        total_ep += ep
        total_fr += fr
        _log(f"  · {s.name}: {ep}ep / {fr}帧")
    _log(f"合计：{len(sessions)} 个数据集 → {total_ep} episode / {total_fr} 帧")

    out_dir = root / args.out
    if args.dry_run:
        _log(f"[dry-run] 将合并到: {out_dir}")
        return 0

    if out_dir.exists():
        if not args.overwrite:
            _log(f"ERROR 输出目录已存在: {out_dir}（加 --overwrite 覆盖）")
            return 1
        _log(f"删除已存在的输出目录: {out_dir}")
        shutil.rmtree(out_dir)

    # merge_datasets 跑在 lerobot 的 python 里（sdg 侧没装 lerobot）
    sys.path.insert(0, str(Path(__file__).resolve().parents[3]))
    from fasim.replicator.sdg import export_lerobot as ex

    py = ex.find_lerobot_python()
    if py is None:
        _log("ERROR 找不到 lerobot python")
        return 1

    _log(f"使用 python: {py}")
    _log("开始合并（数据量大时可能需要几分钟）…")

    # 用子进程执行，避免在 sdg 的 python 里 import lerobot
    code = f'''
import sys, json
from pathlib import Path
from lerobot.datasets.lerobot_dataset import LeRobotDataset
from lerobot.datasets.dataset_tools import merge_datasets

sessions = {str([str(s) for s in sessions])}
out_dir = {str(out_dir)}
out_repo = {args.out!r}

root = out_dir.parent
datasets = []
for s in sessions:
    name = Path(s).name
    datasets.append(LeRobotDataset(repo_id=name, root=Path(s)))
    print(f"[merge] 已加载 {{name}}: {{len(datasets[-1])}} 帧", flush=True)

merged = merge_datasets(
    datasets,
    output_repo_id=out_repo,
    output_dir=out_dir,
)
print(f"[merge] 完成: {{len(merged)}} 帧", flush=True)
'''
    import subprocess

    r = subprocess.run([str(py), "-c", code])
    if r.returncode != 0:
        _log(f"ERROR 合并失败（退出码 {r.returncode}）")
        return 1

    info = out_dir / "meta" / "info.json"
    if info.is_file():
        meta = json.loads(info.read_text(encoding="utf-8"))
        _log(f"✅ 输出: {out_dir}")
        _log(f"   {meta.get('total_episodes')} episode / "
             f"{meta.get('total_frames')} 帧")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
