#!/usr/bin/env python3
"""分析 data-collector-platform 产出的 LeRobot 数据集，与 sdg 同口径对比。

用法:
    python3 analysis_dcp_dataset.py [数据集根目录]

默认目录: ~/Downloads/hole_4_to_hole_3_lerobot/hole_4_to_hole_3

只依赖 numpy + pyarrow（系统 python3 或 lerobot_ros2/.venv 均可）。

输出:
  1. 每条 episode 的帧数 / fps 声明 vs 实测 / 抖动 / 维度 / action 量级
  2. 逐维统计（state 与 action 的 min/max/std）
  3. 四元数模长校验
  4. action 是否 delta（state[t] + action[t] == state[t+1]）
  5. 时间戳字段一致性（timestamp == index/fps）
  6. 运动时间线（每 100 帧的速度统计 + 夹爪状态）
  7. 跨 episode 相似度（判断是否同一条轨迹重放）
  8. pad 维度是否为全零
"""

import json
import sys
from pathlib import Path

import numpy as np

try:
    import pyarrow.parquet as pq
except ImportError:
    sys.exit("需要 pyarrow: pip install pyarrow")

DEFAULT_ROOT = Path.home() / "Downloads/hole_4_to_hole_3_lerobot/hole_4_to_hole_3"


def find_episodes(root: Path):
    """LeRobot 每个 episode 一个目录（同事的布局），兼容标准 lerobot 单目录布局。"""
    eps = sorted(p for p in root.iterdir()
                 if p.is_dir() and "_converted" not in p.name
                 and (p / "data/chunk-000/episode_000000.parquet").is_file())
    return eps


def load_episode(ep: Path):
    t = pq.read_table(ep / "data/chunk-000/episode_000000.parquet")
    d = {c: np.asarray(t.column(c).to_pylist()) for c in t.column_names}
    info_p = ep / "meta/info.json"
    info = json.loads(info_p.read_text()) if info_p.is_file() else {}
    return d, info


def dup_ratio(a):
    """相邻行完全相同的比例。"""
    if len(a) < 2:
        return 0.0
    return float(np.all(a[1:] == a[:-1], axis=1).mean())


def sec(title):
    print(f"\n{'='*78}\n{title}\n{'='*78}")


def main():
    root = Path(sys.argv[1]) if len(sys.argv) > 1 else DEFAULT_ROOT
    if not root.is_dir():
        sys.exit(f"目录不存在: {root}")

    eps = find_episodes(root)
    print(f"数据集: {root}")
    print(f"episodes: {len(eps)}")

    # ---------- 1. 概览 ----------
    sec("1. 概览")
    print(f"{'episode':26s} {'frames':>7s} {'fps声明':>8s} {'fps实测':>8s} "
          f"{'抖动ms':>7s} {'dim':>4s} {'|act|均值':>11s} {'state重复':>10s} {'action重复':>11s}")
    print("-" * 104)
    rows, data = [], {}
    for ep in eps:
        d, info = load_episode(ep)
        st, ac = d["observation.state"], d["action"]
        ts = d.get("capture_timestamp_ns")
        if ts is not None and len(ts) > 1:
            dt = np.diff(ts.astype(np.int64)) / 1e6
            fps_eff = 1000.0 / np.median(dt)
            jit = float(np.abs(dt - np.median(dt)).max())
        else:
            fps_eff = jit = float("nan")
        sr, ar = dup_ratio(st), dup_ratio(ac)
        rows.append(dict(ep=ep.name, n=len(st), fps=info.get("fps"),
                         fps_eff=fps_eff, jit=jit, dim=st.shape[1],
                         act=float(np.abs(ac).mean()), sr=sr, ar=ar))
        data[ep.name] = (d, info)
        print(f"{ep.name:26s} {len(st):7d} {str(info.get('fps')):>8s} "
              f"{fps_eff:8.2f} {jit:7.2f} {st.shape[1]:4d} "
              f"{np.abs(ac).mean():11.5f} {sr*100:9.1f}% {ar*100:10.1f}%")

    if rows:
        n = [r["n"] for r in rows]
        print(f"\n合计 {len(rows)} 条 / {sum(n)} 帧；单条 {min(n)}~{max(n)}（中位 {int(np.median(n))}）")
        print(f"平均 state 重复率 {np.mean([r['sr'] for r in rows])*100:.1f}%  "
              f"平均 action 重复率 {np.mean([r['ar'] for r in rows])*100:.1f}%")

    if not rows:
        sys.exit("没有可分析的 episode")

    # ---------- 2. 时间戳 ----------
    first = rows[0]["ep"]
    d0, info0 = data[first]
    sec(f"2. 时间戳（{first}）")
    ts = d0["capture_timestamp_ns"].astype(np.int64)
    dt = np.diff(ts) / 1e6
    print(f"dt(ms): min={dt.min():.2f} p5={np.percentile(dt,5):.2f} "
          f"med={np.median(dt):.2f} p95={np.percentile(dt,95):.2f} max={dt.max():.2f}")
    vals, cnts = np.unique(np.round(dt, 2), return_counts=True)
    for v, c in sorted(zip(vals, cnts), key=lambda x: -x[1])[:5]:
        print(f"  dt={v:7.2f}ms x{c:5d} ({100*c/len(dt):5.1f}%)")
    print(f"单调递增: {bool(np.all(np.diff(ts) > 0))}；真实时长 {(ts[-1]-ts[0])/1e9:.2f}s")
    print(f"按实测中位间隔推的帧率: {1000/np.median(dt):.2f} Hz")

    fps = info0.get("fps")
    if fps and "timestamp" in d0:
        tsf = np.asarray(d0["timestamp"], dtype=float)
        fi = np.asarray(d0["frame_index"])
        print(f"\ntimestamp 字段 == frame_index/{fps} ? {np.allclose(tsf, fi/fps, atol=1e-6)}")
        real_dur = (ts[-1] - ts[0]) / 1e9
        print(f"声明时长 {len(tsf)/fps:.2f}s  vs  真实时长 {real_dur:.2f}s  "
              f"→ 比值 {len(tsf)/fps/real_dur:.2f}x")

    if "action_timestamp_ns" in d0:
        at = d0["action_timestamp_ns"].astype(np.int64)
        same = bool(len(at) > 1 and np.allclose(at[:-1], ts[1:]))
        print(f"\naction_timestamp[t] == capture_timestamp[t+1] ? {same}")

    # ---------- 3. 逐维统计 ----------
    sec(f"3. 逐维统计（{first}）")
    st, ac = d0["observation.state"], d0["action"]
    feats = info0.get("features", {})
    sn = (feats.get("observation.state", {}).get("names")
          or [f"d{i}" for i in range(st.shape[1])])
    an = (feats.get("action", {}).get("names")
          or [f"d{i}" for i in range(ac.shape[1])])
    print(f"{'dim':18s} {'state min':>11s} {'max':>11s} {'std':>9s} | "
          f"{'act min':>11s} {'max':>11s} {'std':>9s}")
    print("-" * 84)
    for i in range(st.shape[1]):
        print(f"{sn[i]:18s} {st[:,i].min():11.5f} {st[:,i].max():11.5f} {st[:,i].std():9.5f} | "
              f"{ac[:,i].min():11.5f} {ac[:,i].max():11.5f} {ac[:,i].std():9.5f}")

    pads = [i for i, n in enumerate(sn) if str(n).startswith("pad_")]
    if pads:
        dead = [i for i in pads if np.all(st[:, i] == 0) and np.all(ac[:, i] == 0)]
        print(f"\npad 维度: {len(pads)} 个，其中全零 {len(dead)} 个"
              f" → 有效维度 {st.shape[1]-len(dead)}/{st.shape[1]}")

    # ---------- 4. 四元数 ----------
    sec("4. 四元数模长校验")
    for names, arr, kind in ((sn, st, "state"), (an, ac, "action")):
        for i, n in enumerate(names):
            if str(n).endswith(("quat_x", "dquat_x")) and i + 4 <= arr.shape[1]:
                q = arr[:, i:i+4]
                qn = np.linalg.norm(q, axis=1)
                print(f"{kind}[{str(n)[:-2]}*]: norm {qn.min():.4f}~{qn.max():.4f}")

    # ---------- 5. action 是否 delta ----------
    sec("5. action 语义（delta 还是绝对值）")
    pos_pairs = []
    for i, n in enumerate(sn):
        if str(n).endswith("_pos_x"):
            side = str(n)[:-6]
            ai = an.index(f"{side}_dpos_x") if f"{side}_dpos_x" in an else None
            if ai is not None:
                pos_pairs.append((side, i, ai))
    for side, si, ai in pos_pairs:
        pred = st[:-1, si:si+3] + ac[:-1, ai:ai+3]
        err = np.linalg.norm(pred - st[1:, si:si+3], axis=1)
        truth_d = st[1:, si:si+3] - st[:-1, si:si+3]
        print(f"{side:8s} state[t]+action[t] vs state[t+1]: "
              f"mean={err.mean()*1000:.4f}mm max={err.max()*1000:.4f}mm")
        print(f"{'':8s} |action - 真实delta| mean={np.abs(ac[:-1,ai:ai+3]-truth_d).mean()*1000:.4f}mm")
    if not pos_pairs:
        print("（未找到 _pos_x / _dpos_x 配对，跳过）")

    # ---------- 6. 运动时间线 ----------
    sec(f"6. 运动时间线（{first}，每 100 帧）")
    motion_cols = []
    for i, n in enumerate(sn):
        if str(n).endswith("_pos_x"):
            motion_cols.extend(range(i, min(i + 3, st.shape[1])))
    sp = np.zeros(len(st))
    if motion_cols:
        sp[1:] = np.linalg.norm(np.diff(st[:, motion_cols], axis=0), axis=1)
    grip_cols = [i for i, n in enumerate(sn) if str(n).endswith("_gripper")]
    hdr = f"{'frames':>13s} {'mean spd(mm/帧)':>16s} {'max':>8s} {'静止占比':>9s}"
    if grip_cols:
        hdr += "".join(f" {str(sn[i])[:9]:>9s}" for i in grip_cols)
    print(hdr)
    for i in range(0, len(st), 100):
        s = sp[i:i+100]
        line = (f"{i:6d}-{i+len(s):5d} {s.mean()*1000:16.4f} {s.max()*1000:8.2f} "
                f"{(s < 1e-6).mean()*100:8.0f}%")
        if grip_cols:
            line += "".join(f" {st[min(i, len(st)-1), c]:9.2f}" for c in grip_cols)
        print(line)

    # ---------- 7. 跨 episode 相似度 ----------
    if len(rows) > 1:
        sec("7. 跨 episode 相似度（末端位置，重采样到 400 点）")
        base = data[first][0]["observation.state"]
        pos_idx = [i for i, n in enumerate(sn) if str(n).endswith("_pos_x")]
        if pos_idx:
            def rs(a, k=400):
                return a[np.linspace(0, len(a)-1, k).astype(int)]

            def xyz(a, i):
                return a[:, i:i+3]

            print(f"{'episode':26s} {'帧数':>6s} " +
                  " ".join(f"{str(sn[i])[:-6]+'RMS(mm)':>14s}" for i in pos_idx))
            for r in rows[1:]:
                a = data[r["ep"]][0]["observation.state"]
                parts = []
                for i in pos_idx:
                    d = np.sqrt(((xyz(rs(base), i) - xyz(rs(a), i))**2).sum(1).mean()) * 1000
                    parts.append(f"{d:14.2f}")
                print(f"{r['ep']:26s} {len(a):6d} " + " ".join(parts))
            print("\nRMS < 5mm → 同一条轨迹重放；几十 mm → 真实多次演示")

            trav = []
            for r in rows:
                a = data[r["ep"]][0]["observation.state"]
                trav.append((r["ep"], [np.linalg.norm(a[:, i:i+3].max(0) - a[:, i:i+3].min(0))
                                       for i in pos_idx]))
            print(f"\n{'episode':26s} " +
                  " ".join(f"{('行程'+str(sn[i])[:-6]):>16s}" for i in pos_idx))
            for name, ts_ in trav:
                print(f"{name:26s} " + " ".join(f"{t:16.3f}" for t in ts_))

    # ---------- 8. 静止首段 ----------
    sec("8. 静止首段（首个运动帧）")
    for r in rows:
        a = data[r["ep"]][0]["observation.state"]
        if motion_cols and len(a) > 1:
            s = np.zeros(len(a))
            s[1:] = np.linalg.norm(np.diff(a[:, motion_cols], axis=0), axis=1)
            nz = np.flatnonzero(s > 1e-5)
            firstm = int(nz[0]) if len(nz) else len(a)
            if r["fps_eff"] > 0:
                print(f"{r['ep']:26s} 首个运动帧 {firstm:5d} (≈{firstm/r['fps_eff']:.1f}s)")
            else:
                print(f"{r['ep']:26s} 首个运动帧 {firstm:5d}")

    print()


if __name__ == "__main__":
    main()
