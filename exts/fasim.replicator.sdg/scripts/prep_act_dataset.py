#!/usr/bin/env python3
"""ACT 训练前的数据整理：删坏 run → 裁静止段 → 重排序号 → 重新 pack。

为什么要有这个脚本
==================
`trim.py` 是"把一条会话切成多个任务片段"，`prune_idle_prefix.py` 只裁开头。
实际做一次训练前的整理需要一串动作，而且**顺序不能错**（先裁帧再 pack，
否则 episode.npz 与 frames 对不上）。这里把整条链路固化，避免手滑。

四个步骤
========
1. **删除坏 run**：相机分辨率与训练不一致的会话（比如腕部相机被渲染成
   1280x720 而其它是 640x480）。这类数据会让 `capture_profile` 在批量
   合并时拒绝、或让 worker 每帧 feature mismatch。
2. **裁静止段**：把每条会话开头/结尾（可选中间）机器人不动的帧移走。
   判据用**末端位移的中位噪声**自适应，而不是写死 `1e-5`：
   实测静止帧的帧间位移中位数只有 ~1.4e-06 m（亚微米级抖动），
   写死 10µm 会被抖动触发 → 漏裁。
3. **重排序号**：把 `run_XX` 连续重编成 `run_001, run_002, ...`
   （**默认只报告**，因为改目录名会影响你已有的记录习惯；
   加 `--renumber` 才真改）。
4. **重新 pack**：帧变了，旧 `episode.npz` 必须重算（FK 反算末端指令位姿）。

安全性
======
- 默认 **dry-run**，只打印计划，不动任何文件。
- 裁掉的帧移到 `<session>/_idle_prefix/` 与 `<session>/_idle_suffix/`，
  **不删除**，可随时移回。
- 删坏 run 也只是移进任务目录下的 `_quarantine/`，不真删。

用法
====
::

    # 1) 先看计划（什么都不改）
    python prep_act_dataset.py

    # 2) 真做（裁静止段 + pack），坏 run 移进隔离区
    python prep_act_dataset.py --apply

    # 3) 额外：连续重编序号
    python prep_act_dataset.py --apply --renumber

    # 4) 只在某个目录上做
    python prep_act_dataset.py --dir ~/fasim_data/unitree_g1d/sinopec_test_tube --apply
"""

from __future__ import annotations

import argparse
import json
import shutil
import subprocess
import sys
from pathlib import Path

import numpy as np

DEFAULT_DIR = Path.home() / "fasim_data/unitree_g1d/sinopec_test_tube"

#: 期望的相机分辨率，**注意是 (height, width)** —— np 的 ``shape[:2]`` 就是 (H, W)。
#: （曾写成 (640, 480) 导致全部 run 被判不合格。）
EXPECT_HW = (480, 640)

#: 裁静止段时，首尾各额外保留的帧数（让起点"马上要动"，又留出加速段）
PAD_HEAD = 6
PAD_TAIL = 4

#: 静止判据的倍数：帧间位移 < k × 该会话的静止噪声中位数 视为不动。
#: 用中位数而不是均值 —— 均值会被运动帧拉高。
STILL_FACTOR = 8.0

#: 至少连续多少帧才算"一段静止"（20fps 下 10 帧 = 0.5 秒）
MIN_STILL_FRAMES = 10


def _log(msg: str) -> None:
    print(msg, flush=True)


# --------------------------------------------------------------------- 探测


def session_dirs(root: Path) -> list[Path]:
    return sorted(
        p for p in root.iterdir()
        if p.is_dir() and p.name.startswith("run_") and (p / "frames").is_dir()
    )


def frame_files(session: Path) -> list[Path]:
    return sorted((session / "frames").glob("frame_*.npz"))


def check_cameras(session: Path) -> dict:
    """检查该会话所有帧的相机分辨率是否一致且符合期望。

    只抽查首/中/末三帧 —— 分辨率是采集参数，中途不会变；全量扫太慢。
    """
    fs = frame_files(session)
    if not fs:
        return {"ok": False, "reason": "没有帧文件"}
    probe = [fs[0], fs[len(fs) // 2], fs[-1]]
    shapes: dict[str, set] = {}
    for f in probe:
        with np.load(f, allow_pickle=False) as z:
            for c in ("head", "wrist_left", "wrist_right"):
                k = f"{c}_rgb"
                if k not in z.files:
                    shapes.setdefault(c, set()).add("MISSING")
                    continue
                a = np.asarray(z[k])
                shapes.setdefault(c, set()).add(
                    tuple(a.shape[:2]) if a.ndim == 3 else ("EMPTY", a.shape)
                )
    bad: dict[str, list] = {}
    for c, s in shapes.items():
        vals = sorted(str(v) for v in s)
        only = next(iter(s)) if len(s) == 1 else None
        if only != EXPECT_HW:
            bad[c] = vals
    return {"ok": not bad, "bad_cameras": bad, "shapes": shapes}


def ee_track(frames: list[Path]) -> np.ndarray:
    """逐帧两臂末端位置 → (N, 6)。优先实测，缺失用指令位姿。"""
    out = []
    for f in frames:
        with np.load(f, allow_pickle=False) as z:
            keys = set(z.files)
            vals: list[float] = []
            for side in ("left", "right"):
                for cand in (f"{side}_pos", f"{side}_cmd_pos"):
                    if cand in keys:
                        vals.extend(np.asarray(z[cand], dtype=np.float64).reshape(-1)[:3])
                        break
                else:
                    vals.extend([np.nan] * 3)
            out.append(vals)
    return np.asarray(out, dtype=np.float64)


def joint_cmd_track(frames: list[Path]) -> np.ndarray | None:
    """逐帧的**指令**关节角 → (N, J)。优先 ``joint_pos_cmd``。

    为什么判静止要用它而不是末端实测位置
    ------------------------------------
    实测末端的帧间位移有 ~0.2mm 的物理抖动（仿真求解噪声 + 运控保持力），
    拿"末端位移 < 阈值"判静止会被这层噪声淹没；想压过它就得把阈值抬到
    毫米级，于是慢速运动也被判成静止 —— 实测按这个思路会误裁 55%。

    而 ``joint_pos_cmd`` 是编排**直接下发**的目标角，待机时严格不动
    （实测 ``|dq| < 1e-9`` 的帧占绝大部分），一动就是真实运动。
    """
    qs = []
    for f in frames:
        with np.load(f, allow_pickle=False) as z:
            k = "joint_pos_cmd" if "joint_pos_cmd" in z.files else "joint_pos"
            if k not in z.files:
                return None
            qs.append(np.asarray(z[k], dtype=np.float64).reshape(-1))
    if not qs:
        return None
    return np.stack(qs)


#: ``joint_pos_cmd`` 判静止的阈值（rad）：待机帧实测严格为 0，
#: 这里给一点余量以容忍极小的数值噪声。
CMD_STILL_EPS = 1e-9


def still_mask(pos: np.ndarray) -> tuple[np.ndarray, float]:
    """**已弃用**：按末端实测位置判静止会被物理抖动淹没。

    保留仅为兼容旧调用；新代码请用 :func:`still_mask_from_cmd`。
    """
    n = len(pos)
    d = np.zeros(n)
    if n > 1:
        d[1:] = np.linalg.norm(np.diff(pos, axis=0), axis=1)
    finite = d[np.isfinite(d)]
    if not len(finite):
        return np.zeros(n, dtype=bool), 0.0
    noise = float(np.percentile(finite, 25))
    eps = max(noise * STILL_FACTOR, 1e-9)
    still = np.isfinite(d) & (d < eps)
    if len(still):
        still[0] = True
    return still, eps


def still_mask_from_cmd(q: np.ndarray) -> np.ndarray:
    """按**指令关节角**判逐帧是否静止。

    第 i 帧静止 ⇔ 从 i 到 i+1 的指令关节角没有变化（``|dq| < CMD_STILL_EPS``）。
    末帧沿用倒数第二帧的判定（末帧后面没有"下一帧"可比）。
    """
    n = len(q)
    if n == 0:
        return np.zeros(0, dtype=bool)
    if n == 1:
        return np.ones(1, dtype=bool)
    d = np.linalg.norm(np.diff(q, axis=0), axis=1)
    still = d < CMD_STILL_EPS
    still = np.concatenate([still, still[-1:]])   # 末帧沿用前一判定
    still[0] = True if n > 1 and still[0] else still[0]
    return still


def find_idle_blocks(still: np.ndarray, min_len: int) -> list[tuple[int, int]]:
    """连续静止块 [(start, end_inclusive), ...]。"""
    out: list[tuple[int, int]] = []
    s = None
    for i, v in enumerate(still):
        if v and s is None:
            s = i
        elif not v and s is not None:
            if i - s >= min_len:
                out.append((s, i - 1))
            s = None
    if s is not None and len(still) - s >= min_len:
        out.append((s, len(still) - 1))
    return out


# --------------------------------------------------------------------- 动作


def plan_session(session: Path) -> dict:
    """给出该会话的裁剪计划（不改文件）。"""
    fs = frame_files(session)
    if len(fs) < MIN_STILL_FRAMES * 2:
        return {"session": session.name, "skip": "帧太少", "n": len(fs)}

    q = joint_cmd_track(fs)
    if q is None:
        return {"session": session.name, "skip": "没有 joint_pos_cmd / joint_pos", "n": len(fs)}

    still = still_mask_from_cmd(q)
    blocks = find_idle_blocks(still, MIN_STILL_FRAMES)
    n = len(fs)

    drop = np.zeros(n, dtype=bool)
    head_drop = tail_drop = mid_drop = 0
    for s, e in blocks:
        s2 = max(0, s)
        e2 = min(n - 1, e)
        is_head = s2 == 0
        is_tail = e2 == n - 1
        if is_head and is_tail:
            # 整段都静止 —— 这是废数据，但不在本脚本职责内删；不动
            continue
        if is_head:
            cut = max(0, e2 + 1 - PAD_HEAD)     # 保留最后 PAD_HEAD 帧
            drop[:cut] = True
            head_drop += cut
        elif is_tail:
            cut_from = min(n, s2 + PAD_TAIL)    # 保留最前 PAD_TAIL 帧
            drop[cut_from:] = True
            tail_drop += n - cut_from
        else:
            drop[s2:e2 + 1] = True
            mid_drop += e2 - s2 + 1

    keep = int((~drop).sum())
    return {
        "session": session.name,
        "n": n,
        "kept": keep,
        "drop_head": head_drop,
        "drop_tail": tail_drop,
        "drop_mid": mid_drop,
        "drop_total": int(drop.sum()),
        "eps": CMD_STILL_EPS,
        "idle_blocks": blocks,
        "_drop": drop,
        "_frames": fs,
    }


def apply_session(plan: dict, session: Path) -> dict:
    """执行裁剪：把要丢的帧移进 _idle_prefix / _idle_suffix，重算 pack。"""
    fs: list[Path] = plan["_frames"]
    drop: np.ndarray = plan["_drop"]
    n = len(fs)
    info = {"session": session.name}

    if not drop.any():
        info["note"] = "无静止段，跳过"
        return info

    pre = session / "_idle_prefix"
    suf = session / "_idle_suffix"
    mid = session / "_idle_mid"
    n_moved = 0
    for i, f in enumerate(fs):
        if not drop[i]:
            continue
        if i < n // 3:
            dst = pre
        elif i > (2 * n) // 3:
            dst = suf
        else:
            dst = mid
        dst.mkdir(exist_ok=True)
        target = dst / f.name
        if target.exists():          # 多次执行时避免覆盖
            k = 1
            while (dst / f"{f.stem}__{k}{f.suffix}").exists():
                k += 1
            target = dst / f"{f.stem}__{k}{f.suffix}"
        shutil.move(str(f), str(target))
        n_moved += 1
    info["moved"] = n_moved

    # session.json 记账
    sj = session / "session.json"
    if sj.is_file():
        try:
            m = json.loads(sj.read_text(encoding="utf-8"))
            kept = n - n_moved
            m["frames_saved"] = kept
            m["physics_steps"] = kept * int(m.get("every", 10) or 10)
            m["idle_trim"] = {
                "dropped": n_moved,
                "kept": kept,
                "threshold_eps_m": plan["eps"],
                "note": "静止段已裁，原帧在 _idle_prefix/ _idle_suffix/ _idle_mid/",
            }
            sj.write_text(json.dumps(m, indent=2, ensure_ascii=False), encoding="utf-8")
        except Exception as exc:  # noqa: BLE001
            info["warn"] = f"session.json 更新失败: {exc}"

    # 帧变了 → 旧 episode.npz 失效
    ep = session / "episode.npz"
    if ep.is_file():
        ep.unlink()
        info["removed_episode_npz"] = True
    return info


def renumber(root: Path, sessions: list[Path], apply: bool) -> list[tuple[str, str]]:
    """把 run_XX 连续重编为 run_001...（两阶段改名避免碰撞）。"""
    mapping = [(s.name, f"run_{i:03d}") for i, s in enumerate(sessions, 1)]
    changes = [(a, b) for a, b in mapping if a != b]
    if not apply or not changes:
        return changes
    tmp = []
    for i, (old, new) in enumerate(changes):
        t = root / f".__renum_{i:04d}__"
        (root / old).rename(t)
        tmp.append((t, new))
    for t, new in tmp:
        t.rename(root / new)
    return changes


def run_pack(root: Path, python: str | None = None) -> int:
    """调用 sdg.sh 的批量 pack。"""
    sdg = Path(__file__).resolve().parent / "sdg.sh"
    cmd = ["bash", str(sdg), "pack", str(root), "-y"]
    if python:
        cmd = ["bash", str(sdg), "pack", str(root), "--lerobot-python", python, "-y"]
    _log(f"\n[pack] {' '.join(cmd)}")
    return subprocess.run(cmd).returncode


def main() -> int:
    ap = argparse.ArgumentParser(description="ACT 训练前数据整理")
    ap.add_argument("--dir", default=str(DEFAULT_DIR), help="任务数据目录")
    ap.add_argument("--apply", action="store_true", help="真的执行（默认只报告）")
    ap.add_argument("--renumber", action="store_true", help="连续重编序号 run_001...")
    ap.add_argument("--no-pack", action="store_true", help="不自动 pack")
    ap.add_argument("--quarantine-bad", action="store_true",
                    help="把相机分辨率不合格的 run 移进 _quarantine/")
    args = ap.parse_args()

    root = Path(args.dir).expanduser().resolve()
    if not root.is_dir():
        _log(f"目录不存在: {root}")
        return 2
    sessions = session_dirs(root)
    if not sessions:
        _log(f"{root} 下没有 run_* 会话")
        return 2

    _log(f"目录: {root}")
    _log(f"会话: {len(sessions)} 个\n")

    # ---- 1) 相机分辨率检查 ----
    _log("=" * 78)
    _log("1) 相机分辨率检查（期望三路都是 640x480）")
    _log("=" * 78)
    bad_runs: list[Path] = []
    for s in sessions:
        r = check_cameras(s)
        if not r["ok"]:
            bad_runs.append(s)
            detail = ", ".join(f"{c}={v}" for c, v in r["bad_cameras"].items())
            _log(f"  ✗ {s.name:10s} {detail}")
    if not bad_runs:
        _log("  全部合格")
    else:
        _log(f"\n  共 {len(bad_runs)} 条不合格: {[p.name for p in bad_runs]}")
        if args.quarantine_bad and args.apply:
            q = root / "_quarantine"
            q.mkdir(exist_ok=True)
            for s in bad_runs:
                dst = q / s.name
                if dst.exists():
                    _log(f"  跳过（已在隔离区）: {s.name}")
                    continue
                shutil.move(str(s), str(dst))
                _log(f"  → 已移入 {q.name}/{s.name}")
            sessions = [s for s in sessions if s not in bad_runs]
        elif args.quarantine_bad:
            _log("  （dry-run：加 --apply 才会移进 _quarantine/）")
        else:
            _log("  提示：加 --quarantine-bad 可把这些 run 移进 _quarantine/")

    # ---- 2) 静止段裁剪计划 ----
    _log("\n" + "=" * 78)
    _log("2) 静止段裁剪（只裁首尾；中段静止仅报告不动，避免切碎任务）")
    _log("=" * 78)
    plans = []
    _log(f"  {'run':10s} {'原帧':>5s} {'裁头':>5s} {'裁尾':>5s} {'中段':>5s} {'保留':>5s} {'省':>6s}")
    _log("  " + "-" * 62)
    for s in sessions:
        p = plan_session(s)
        if "skip" in p:
            _log(f"  {p['session']:10s} 跳过：{p['skip']}")
            continue
        plans.append((s, p))
        pct = 100.0 * p["drop_total"] / max(p["n"], 1)
        _log(f"  {p['session']:10s} {p['n']:5d} {p['drop_head']:5d} "
             f"{p['drop_tail']:5d} {p['drop_mid']:5d} {p['kept']:5d} {pct:5.1f}%")
    if plans:
        tot_n = sum(p["n"] for _, p in plans)
        tot_k = sum(p["kept"] for _, p in plans)
        _log("  " + "-" * 62)
        _log(f"  合计 {tot_n} → {tot_k} 帧（裁掉 {tot_n - tot_k}，"
             f"{100.0*(tot_n-tot_k)/max(tot_n,1):.1f}%）")

    if not args.apply:
        _log("\n这是试运行，未修改任何文件。确认无误后加 --apply。")
        if plans:
            _log("（裁掉的帧会移进各自会话的 _idle_prefix/ _idle_suffix/ _idle_mid/，不删除）")
        return 0

    # ---- 3) 执行裁剪 ----
    _log("\n" + "=" * 78)
    _log("3) 执行裁剪")
    _log("=" * 78)
    changed = 0
    for s, p in plans:
        r = apply_session(p, s)
        if r.get("moved"):
            changed += 1
            _log(f"  {s.name:10s} 移走 {r['moved']} 帧，保留 {p['kept']}")
    _log(f"  共 {changed} 条会话有改动")

    # ---- 4) 重排序号 ----
    if args.renumber:
        _log("\n" + "=" * 78)
        _log("4) 连续重编序号")
        _log("=" * 78)
        fresh = session_dirs(root)
        ch = renumber(root, fresh, apply=True)
        if not ch:
            _log("  序号已连续，无需改动")
        else:
            for a, b in ch[:12]:
                _log(f"  {a} → {b}")
            if len(ch) > 12:
                _log(f"  ... 共 {len(ch)} 条")

    # ---- 5) 重新 pack ----
    if not args.no_pack:
        _log("\n" + "=" * 78)
        _log("5) 重新 pack（帧变了，episode.npz 需重算 FK 末端指令位姿）")
        _log("=" * 78)
        rc = run_pack(root)
        if rc != 0:
            _log(f"[WARN] pack 退出码 {rc} —— 检查上面的报错")
            return rc

    _log("\n完成。下一步：")
    _log(f"  bash {Path(__file__).resolve().parent}/sdg.sh export <任务目录> \\")
    _log("      --out ~/lerobot_datasets/tube_ee_grip --action-space ee_absolute_gripper --overwrite")
    return 0


if __name__ == "__main__":
    sys.exit(main())
