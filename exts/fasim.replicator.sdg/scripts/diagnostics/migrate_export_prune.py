"""批量把已有采集包导出成 lerobot 并裁剪 frames（一次性迁移脚本）。

用途：在 auto 流程加入「自动导出+裁剪」之前采集的包，需要补做一遍。
     之后 auto 会自动完成，这个脚本只是历史数据迁移用。

安全保证：
  - 先导出 lerobot，**成功才**删 frames（失败则整个包保持原样）
  - 已有 lerobot 数据集且完整时跳过导出，直接裁剪
  - 每个包独立 try/except，一个失败不影响其余
"""

from __future__ import annotations

import os
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[3]))

from fasim.replicator.sdg.auto_collect import export_to_lerobot, prune_frames


def main() -> int:
    task_dir = Path(
        os.environ.get(
            "TASK_DIR",
            str(Path.home() / "fasim_data/unitree_g1d/sinopec_test_tube"),
        )
    ).expanduser()
    only = os.environ.get("ONLY", "").strip()
    keep = int(os.environ.get("KEEP", "8"))
    robot_type = os.environ.get("ROBOT_TYPE", "unitree_g1d")

    sessions = sorted(
        (p for p in task_dir.glob("run_*") if (p / "frames").is_dir()
         and len(list((p / "frames").glob("frame_*.npz"))) > keep),
        key=lambda p: int(p.name.split("_")[1]),
    )
    if only:
        want = {x.strip() for x in only.split(",")}
        sessions = [s for s in sessions if s.name in want]

    print(f"待处理 {len(sessions)} 个包（keep={keep}）")
    for s in sessions:
        print(f"  · {s.name}")

    ok = fail = 0
    for i, s in enumerate(sessions, 1):
        print()
        print("=" * 62)
        print(f"[{i}/{len(sessions)}] {s.name}")
        print("=" * 62)
        try:
            # 1) 导出（已有完整数据集则跳过）
            ds = Path.home() / "lerobot_datasets" / s.name
            already = (ds / "meta" / "info.json").is_file()
            if already:
                print(f"  已有数据集 {ds.name}，跳过导出")
            else:
                res = export_to_lerobot(str(s), robot_type=robot_type)
                if res.get("error"):
                    print(f"  ✗ 导出失败，保留 frames: {res['error']}")
                    fail += 1
                    continue

            # 2) 裁剪
            pr = prune_frames(str(s), keep=keep)
            print(f"  ✓ 删除 {pr['deleted']} 帧，释放 {pr['freed_mb']}MB，"
                  f"保留 {len(pr['kept'])} 帧")
            ok += 1
        except Exception as exc:  # noqa: BLE001
            print(f"  ✗ 异常: {exc}")
            fail += 1

    print()
    print("=" * 62)
    print(f"完成: 成功 {ok}，失败 {fail}")
    return 0 if fail == 0 else 1


if __name__ == "__main__":
    raise SystemExit(main())
