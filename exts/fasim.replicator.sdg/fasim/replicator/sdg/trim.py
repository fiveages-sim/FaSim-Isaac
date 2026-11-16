"""把一次连续采集切成若干「任务片段」—— 剔除待机段。

为什么需要
==========
``sdg.sh capture`` 从你按下确认就开始录，到你 ``stop`` 才停。中间机器人
可能有大量待机（编排启动前的准备、两遍任务之间的空档）。

这些待机段对 ACT 训练是**有害的**：obs 几乎不变、action 也几乎不变，
模型会学到"看到静止画面就不动"这个退化解，导致真跑时该动也不动。

判据
====
用**关节角变化率**：相邻帧关节角的 L2 差值 / dt。静止时接近 0（仿真里
有微小抖动，实测 ~1e-5 rad/s），一旦编排开始动，立刻跳到 0.1~1 rad/s 量级。

不依赖任何外部信号（ROS 时间戳、运控状态），纯看数据本身。

用法
====
::

    sdg.sh trim <session>                       # 只看报告，不写文件
    sdg.sh trim <session> --write               # 切分并写出多个 episode
    sdg.sh trim <session> --threshold 0.02 --pad 1.0

产物（``--write``）
===================
::

    <session>/segments/
      segment_001.npz     episode 格式，可直接 preview / 训练
      segment_002.npz
      ...
    <session>/segments/trim_report.json
"""

from __future__ import annotations

import json
from pathlib import Path
from typing import Any

import numpy as np


def _log(msg: str) -> None:
    print(f"[trim] {msg}", flush=True)


def load_tracks(session_dir: str | Path) -> dict[str, Any]:
    """把会话里的关节/末端/物体轨迹整理成数组（不读图像）。"""
    d = Path(session_dir).expanduser().resolve()
    frames = sorted((d / "frames").glob("frame_*.npz"))
    if not frames:
        raise ValueError(f"没有帧文件: {d / 'frames'}")

    joint_pos: list[np.ndarray] = []
    joint_names: list[str] | None = None
    sim_steps: list[int] = []
    extras: dict[str, list[np.ndarray]] = {}

    for i, f in enumerate(frames):
        with np.load(f, allow_pickle=False) as z:
            keys = set(z.files)
            if "joint_pos" in keys:
                joint_pos.append(np.asarray(z["joint_pos"], dtype=np.float64).reshape(-1))
                if joint_names is None and "joint_names" in keys:
                    joint_names = [str(x) for x in z["joint_names"]]
            sim_steps.append(int(z["sim_step"]) if "sim_step" in keys else i)
            for k in keys:
                if k == "joint_pos" or k.startswith("obj/"):
                    extras.setdefault(k, [])
                if (k.endswith("_pos") or k.endswith("_quat")) and k != "joint_pos":
                    extras.setdefault(k, []).append(
                        np.asarray(z[k], dtype=np.float64).reshape(-1)
                    )

    if not joint_pos:
        raise ValueError("没有 joint_pos（采集时关了 record_joints？trim 需要关节角）")

    steps = np.asarray(sim_steps, dtype=np.int64)
    # 帧间隔（秒）：sim_step 差值 × **设定**物理步长。
    #
    # 注意 physics_dt 是**设定值**（场景 physxScene:timeStepsPerSecond 决定，
    # 试管场景 200Hz=5ms），不是墙钟实际步长。墙钟受机器性能限制（本机约
    # 177Hz），但它只影响跑多快，**不影响仿真时间轴** —— 拿它换算会整体偏。
    phys_dt = 0.0
    try:
        sj = d / "session.json"
        if sj.is_file():
            meta = json.loads(sj.read_text(encoding="utf-8"))
            phys_dt = float(meta.get("physics_dt", 0) or 0)
            # 次选：任务配置声明的设定值（physics_hz → dt）
            if phys_dt <= 0 and (meta.get("physics_hz_setting") or 0) > 0:
                phys_dt = 1.0 / float(meta["physics_hz_setting"])
            if phys_dt <= 0 and meta.get("robot_key") and meta.get("task_key"):
                try:
                    from .config_io import load_task_config

                    hz = load_task_config(meta["robot_key"], meta["task_key"]).physics_hz
                    phys_dt = 1.0 / float(hz) if hz else 0.0
                except Exception:  # noqa: BLE001
                    pass
            # **不要**用 duration_sec / physics_steps 反推：那算的是墙钟实际
            # 步长（本机约 177Hz），而时间轴要用设定值（200Hz），差 13%。
    except Exception:  # noqa: BLE001
        pass
    if phys_dt <= 0:
        phys_dt = 0.005
        _log("WARN 没有 physics_dt，退回假设 5ms（时间轴可能偏）")

    if len(steps) >= 2:
        dstep = float(np.median(np.diff(steps)))
        dt = dstep * phys_dt if dstep > 0 else phys_dt * 10
    else:
        dt = phys_dt * 10
    _log(f"帧间隔: {dt*1000:.2f} ms（物理步长 {phys_dt*1000:.3f}ms）")

    return {
        "dir": d,
        "frames": frames,
        "joint_pos": np.stack(joint_pos),
        "joint_names": joint_names or [],
        "dt": dt,
        "extras": {k: np.stack(v) for k, v in extras.items() if v},
    }


def compute_activity(q: np.ndarray, dt: float) -> np.ndarray:
    """每帧的关节角变化率（rad/s）。首帧记 0。"""
    if q.shape[0] < 2:
        return np.zeros(q.shape[0])
    dq = np.linalg.norm(np.diff(q, axis=0), axis=1) / max(dt, 1e-9)
    return np.concatenate([[0.0], dq])


def find_segments(
    activity: np.ndarray,
    threshold: float,
    pad_sec: float,
    dt: float,
    min_len_sec: float,
    merge_gap_sec: float,
) -> list[tuple[int, int]]:
    """找出活跃区间 [(start, end)]，含前后缓冲与短间隔合并。

    Args:
        activity: 每帧变化率。
        threshold: 超过此值算"在动"（rad/s）。
        pad_sec: 每段前后各留多少秒缓冲（防止漏掉启动/收尾）。
        dt: 帧间隔。
        min_len_sec: 短于此长度的段丢弃（噪声）。
        merge_gap_sec: 两段间隔短于此值时合并（任务中途短暂停顿）。
    """
    active = activity > threshold
    if not active.any():
        return []

    segments: list[list[int]] = []
    start = None
    for i, a in enumerate(active):
        if a and start is None:
            start = i
        elif not a and start is not None:
            segments.append([start, i - 1])
            start = None
    if start is not None:
        segments.append([start, len(active) - 1])

    # 合并短间隔
    gap = max(1, int(merge_gap_sec / dt))
    merged: list[list[int]] = []
    for s, e in segments:
        if merged and s - merged[-1][1] <= gap:
            merged[-1][1] = e
        else:
            merged.append([s, e])

    # 加缓冲 + 过滤太短
    pad = max(0, int(pad_sec / dt))
    out: list[tuple[int, int]] = []
    n = len(active)
    for s, e in merged:
        s2 = max(0, s - pad)
        e2 = min(n - 1, e + pad)
        if (e2 - s2) * dt >= min_len_sec:
            out.append((s2, e2))
    return out


def find_whole_span(
    activity: np.ndarray,
    threshold: float,
    dt: float,
    head_sec: float,
    tail_sec: float,
) -> tuple[int, int] | None:
    """整段模式：找出「保留开头一点 home + 完整任务 + 回 home 结束」的区间。

    与 :func:`find_segments` 的区别（这是数采训练用的，不是切分工具）：

    - ``find_segments`` 找**所有活跃段**并各切一条 —— 适合一个 session 里
      跑了多个任务的情况。
    - 本函数**整段保留**，只砍掉开头过长的待机和结尾多余的待机 —— 适合
      「一个 session = 一条完整 episode」的数采，且**必须保留动作连续性**
      （开头 home 姿态、结尾回 home 的过程都是训练要学的动作）。

    为什么要保留开头一小段 home
    ---------------------------
    编排开始前仿真可能已经 Play 了十几秒（等 OCS2 就绪），这段全静止，
    对训练无价值（ACT 会学到"长期不动"）。但**完全从第一帧动作开始切**
    又会让 episode 缺少"起始于 home 姿态"这个上下文 —— 策略需要知道
    "从待命状态开始"。所以保留 ``head_sec`` 秒静止作为起点。

    为什么结尾要截到「最后一次明显运动之后一点」
    -------------------------------------------
    回 home 的过程是训练要学的收尾动作，必须保留；但回 home 完成后
    还会有几秒静止（等下一次 stop），那部分无价值。所以取
    「最后一次活跃帧 + ``tail_sec``」作为结尾 —— ``tail_sec`` 让机器人
    真正停稳，避免截断在插值中途。

    Args:
        activity: 每帧关节变化率（rad/s）。
        threshold: 超过此值算"在动"。
        dt: 帧间隔（秒）。
        head_sec: 开头保留多少秒静止（home 姿态）。
        tail_sec: 结尾在最后一次运动后多留多少秒（等停稳）。

    Returns:
        ``(start, end)`` 闭区间；全程无活动返回 None。
    """
    active = activity > threshold
    idx = np.flatnonzero(active)
    if idx.size == 0:
        return None

    first_active = int(idx[0])
    last_active = int(idx[-1])

    head_pad = max(0, int(round(head_sec / dt)))
    tail_pad = max(0, int(round(tail_sec / dt)))

    start = max(0, first_active - head_pad)
    end = min(len(activity) - 1, last_active + tail_pad)
    return start, end


def trim_session(
    session_dir: str | Path,
    threshold: float = 0.01,
    pad_sec: float = 1.0,
    min_len_sec: float = 2.0,
    merge_gap_sec: float = 1.5,
    write: bool = False,
    mode: str = "segments",
    head_sec: float = 2.0,
    tail_sec: float = 1.5,
    out_name: str = "episode_trimmed",
) -> dict:
    """主入口。返回报告 dict；``write=True`` 时写出 npz。

    Args:
        mode: ``"segments"`` = 旧行为（找所有活跃段各切一条，适合一个
            session 里跑了多个任务）；``"whole"`` = 整段保留、只裁掉
            开头过长待机和结尾多余待机（**数采训练用这个**）。
        head_sec: whole 模式下开头保留多少秒静止（home 姿态）。
        tail_sec: whole 模式下结尾在最后一次运动后多留多少秒。
        out_name: whole 模式写出的文件名（不含 .npz）。
    """
    data = load_tracks(session_dir)
    q = data["joint_pos"]
    dt = data["dt"]
    n = q.shape[0]

    act = compute_activity(q, dt)

    if mode == "whole":
        span = find_whole_span(act, threshold, dt, head_sec, tail_sec)
        segs = [span] if span else []
    else:
        segs = find_segments(act, threshold, pad_sec, dt, min_len_sec, merge_gap_sec)

    report = {
        "session": str(data["dir"]),
        "mode": mode,
        "frames_total": n,
        "dt_ms": round(dt * 1000, 2),
        "duration_sec": round(n * dt, 2),
        "threshold_rad_s": threshold,
        "activity": {
            "max": round(float(act.max()), 4),
            "p95": round(float(np.percentile(act, 95)), 4),
            "median": round(float(np.median(act)), 6),
        },
        "segments": [
            {
                "index": i + 1,
                "start_frame": int(s),
                "end_frame": int(e),
                "start_sec": round(s * dt, 2),
                "end_sec": round(e * dt, 2),
                "frames": int(e - s + 1),
                "duration_sec": round((e - s + 1) * dt, 2),
            }
            for i, (s, e) in enumerate(segs)
        ],
    }
    if mode == "whole":
        report["head_sec"] = head_sec
        report["tail_sec"] = tail_sec
    else:
        report["pad_sec"] = pad_sec

    kept = sum(s["frames"] for s in report["segments"])
    report["kept_frames"] = kept
    report["kept_ratio"] = round(kept / n, 3) if n else 0.0
    report["dropped_frames"] = n - kept

    _log(f"{n} 帧 / {report['duration_sec']}s，dt={report['dt_ms']}ms  [{mode}]")
    _log(f"活跃度 max={report['activity']['max']} rad/s  median={report['activity']['median']}")
    if not segs:
        _log("WARN 没找到任何活跃段 —— 阈值太大？试试 --threshold 0.005")
    for s in report["segments"]:
        _log(f"  段 {s['index']}: 帧 {s['start_frame']}-{s['end_frame']} "
             f"({s['start_sec']}s-{s['end_sec']}s) {s['duration_sec']}s")
    if mode == "whole" and segs:
        s0 = report["segments"][0]
        _log(f"  裁掉：开头 {s0['start_frame']} 帧（{s0['start_sec']}s 待机），"
             f"结尾 {n - 1 - s0['end_frame']} 帧"
             f"（{round((n - 1 - s0['end_frame']) * dt, 2)}s）")
    _log(f"保留 {kept}/{n} 帧（{report['kept_ratio']*100:.1f}%）")

    out_dir = data["dir"] / "segments"
    if write and segs:
        out_dir.mkdir(parents=True, exist_ok=True)
        if mode == "whole":
            # 整段模式只出一个文件，直接放会话根目录（下游 pack/导出更好找）
            out_path = data["dir"] / f"{out_name}.npz"
            _write_segment(data, segs[0][0], segs[0][1], out_path, kind="whole_trimmed")
            report["written"] = str(out_path)
            _log(f"已写出 → {out_path}")
        else:
            for i, (s, e) in enumerate(segs, 1):
                _write_segment(data, s, e, out_dir / f"segment_{i:03d}.npz")
            _log(f"已写出 {len(segs)} 个 episode → {out_dir}")
        (out_dir / "trim_report.json").write_text(
            json.dumps(report, indent=2, ensure_ascii=False), encoding="utf-8"
        )
        _log(f"报告: {out_dir / 'trim_report.json'}")
    elif write:
        _log("没有段可写")
    else:
        _log("（只报告，未写文件；加 --write 才会写出）")

    return report


def _write_segment(
    data: dict, s: int, e: int, out: Path, kind: str = "live_capture_trimmed"
) -> Path:
    """写一个 episode 格式的 npz（与 pack 的输出一致）。

    Args:
        kind: 写进 meta 的来源标记。``live_capture_trimmed`` = 从长 session
            切出的段；``whole_trimmed`` = 整段保留、只裁两头待机。
    """
    q = data["joint_pos"][s:e + 1]
    n = q.shape[0]
    dt = data["dt"]
    names = data["joint_names"] or [f"j{i}" for i in range(q.shape[1])]

    payload: dict[str, Any] = {
        "schema_version": np.array(2),
        "t": np.arange(n, dtype=np.float64) * dt,
        "joint_pos": q,
        "joint_names": np.array(names),
        "frame_id": np.array("world"),
        "meta": np.array(
            json.dumps(
                {
                    "kind": kind,
                    "source_session": str(data["dir"]),
                    "start_frame": int(s),
                    "end_frame": int(e),
                    "frames": n,
                    "frame_dt": dt,
                },
                ensure_ascii=False,
            )
        ),
    }
    # 末端/物体：有就带上（这些不参与活跃度判定，但要跟着切）
    for side in ("left", "right"):
        p = data["extras"].get(f"{side}_pos")
        qq = data["extras"].get(f"{side}_quat")
        if p is not None and qq is not None and len(p) >= e + 1:
            payload[f"{side}_pos"] = p[s:e + 1]
            payload[f"{side}_quat"] = qq[s:e + 1]
        else:
            payload[f"{side}_pos"] = np.full((n, 3), np.nan)
            payload[f"{side}_quat"] = np.full((n, 4), np.nan)
    for k, v in data["extras"].items():
        if k.startswith("obj/") and len(v) >= e + 1:
            payload[k] = v[s:e + 1]

    np.savez_compressed(out, **payload)
    return out
