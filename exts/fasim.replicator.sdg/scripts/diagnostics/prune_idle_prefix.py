"""裁掉每条采集会话开头的「静止等待段」，让策略学到"开局就动"。

## 为什么
实测 39 条轨迹：首个末端位移帧的中位数是 **119 帧（6.0 秒）**，最长 16.2 秒；
静止帧平均占 30%。而 ACT 的输入是**单帧观测**（`n_obs_steps=1`），
初始画面和"等待中"的画面几乎无差别 —— 模型无法区分"该等"还是"该动"，
训练数据里这种画面后面跟的都是"不动"，于是学到"开局保持静止"。

闭环时形成死循环：看到初始姿态 → 输出不动 → 观测不变 → 永远不动。

## 做法
1. 找到首个末端位移 > 0.01mm 的帧 `first_move`
2. 保留 `first_move - PAD_FRAMES` 之后的帧（留一点"即将动"的缓冲）
3. 前面的帧移到 `_idle_prefix/`（**不删，可恢复**）
4. 更新 session.json 的 frames_saved / physics_steps
5. 删除旧 episode.npz（帧变了要重新 pack）

## 用法
    python prune_idle_prefix.py --dry-run     # 只看会裁多少
    python prune_idle_prefix.py --apply       # 真裁
"""

from __future__ import annotations

import argparse
import json
import shutil
from pathlib import Path

import numpy as np

TASK_DIR = Path.home() / "fasim_data/unitree_g1d/sinopec_test_tube"

#: 首个运动帧之前额外保留的帧数 —— 让 episode 起点是「马上要动」而不是
#: 「已经动了一半」。太大会重新引入等待，太小会丢掉动作起始的加速段。
PAD_FRAMES = 8

#: 判定「在动」的阈值：末端位置相邻帧位移（米）。0.01mm 足以滤掉数值噪声，
#: 又不会漏掉缓慢起步。
MOVE_EPS = 1e-5


def ee_track(frames: list[Path]) -> np.ndarray:
    """按帧序读两臂末端位置，返回 (N, 6)。优先用实测值（真值），缺失用指令位姿。"""
    out = []
    for f in frames:
        with np.load(f, allow_pickle=False) as z:
            keys = set(z.files)
            vals = []
            for side in ("left", "right"):
                for cand in (f"{side}_pos", f"{side}_cmd_pos"):
                    if cand in keys:
                        vals.extend(np.asarray(z[cand]).reshape(-1)[:3])
                        break
                else:
                    vals.extend([np.nan, np.nan, np.nan])
            out.append(vals)
    return np.asarray(out, dtype=np.float64)


def prune(session: Path, *, apply: bool) -> dict | None:
    fd = session / "frames"
    frames = sorted(fd.glob("frame_*.npz")) if fd.is_dir() else []
    if len(frames) < 10:
        return None
    pos = ee_track(frames)
    if not np.isfinite(pos).any():
        return None

    d = np.zeros(len(pos))
    d[1:] = np.linalg.norm(np.diff(pos, axis=0), axis=1)
    moving = d > MOVE_EPS
    if not moving.any():
        return {"session": session.name, "n": len(frames), "first_move": len(frames),
                "keep_from": len(frames), "dropped": 0, "note": "全程静止，跳过"}
    first_move = int(np.argmax(moving))
    keep_from = max(0, first_move - PAD_FRAMES)
    drop = frames[:keep_from]

    info = {
        "session": session.name,
        "n": len(frames),
        "first_move": first_move,
        "first_move_sec": round(first_move / 20.0, 1),
        "keep_from": keep_from,
        "dropped": len(drop),
        "kept": len(frames) - len(drop),
    }
    if not apply or not drop:
        return info

    # 移到 _idle_prefix/（不删，可恢复）
    dst = session / "_idle_prefix"
    dst.mkdir(exist_ok=True)
    for f in drop:
        shutil.move(str(f), str(dst / f.name))

    # 更新 session.json
    sj = session / "session.json"
    if sj.is_file():
        try:
            m = json.loads(sj.read_text(encoding="utf-8"))
            kept = len(frames) - len(drop)
            m["frames_saved"] = kept
            m["physics_steps"] = kept * int(m.get("every", 10) or 10)
            m["idle_prefix_pruned"] = {
                "dropped_frames": len(drop),
                "first_move_frame": first_move,
                "kept_from_original": keep_from,
                "note": "开头静止等待段已裁掉，原帧在 _idle_prefix/",
            }
            sj.write_text(json.dumps(m, indent=2, ensure_ascii=False), encoding="utf-8")
        except Exception as exc:  # noqa: BLE001
            info["warn"] = f"session.json 更新失败: {exc}"

    # 帧变了，旧 episode.npz 失效
    ep = session / "episode.npz"
    if ep.is_file():
        ep.unlink()
        info["removed_episode_npz"] = True
    return info


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--apply", action="store_true", help="真的裁剪（默认只报告）")
    ap.add_argument("--dir", default=str(TASK_DIR))
    args = ap.parse_args()

    root = Path(args.dir).expanduser()
    sessions = sorted(p for p in root.iterdir() if p.is_dir())
    rows = []
    for s in sessions:
        r = prune(s, apply=args.apply)
        if r:
            rows.append(r)

    if not rows:
        print("没有可处理的会话")
        return 1

    print(f"{'session':10s} {'原帧':>5s} {'首动':>5s} {'首动s':>6s} {'裁掉':>5s} {'保留':>5s}")
    print("-" * 50)
    for r in rows:
        print(f"{r['session']:10s} {r['n']:5d} {r['first_move']:5d} "
              f"{r.get('first_move_sec', 0):6.1f} {r['dropped']:5d} {r['kept']:5d}")
    tot_before = sum(r["n"] for r in rows)
    tot_after = sum(r["kept"] for r in rows)
    print("-" * 50)
    print(f"合计: {tot_before} → {tot_after} 帧 "
          f"（裁掉 {tot_before - tot_after}，{100*(tot_before-tot_after)/max(tot_before,1):.0f}%）")
    print()
    if not args.apply:
        print("这是试运行。加 --apply 才会真的移动文件。")
    else:
        print("已完成。原帧在各自的 _idle_prefix/ 下，需要恢复就地移回 frames/。")
        print("下一步：sdg.sh pack <任务目录> -y  然后 export / 训练")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
