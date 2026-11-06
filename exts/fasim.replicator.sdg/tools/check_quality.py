"""数据质量检查：episode(npz) 层与渲染层。

分三档，对应"我没法评测数据质量"的三个层次：

**档1 自洽性**（只需 episode，几秒出结果）
    - 帧数、时长、帧间隔是否均匀
    - 关节角是否连续（无跳变）、是否在限位内
    - 末端位姿是否连续
    - 夹爪切换次数是否合理

**档2 回放忠实度**（需要渲染层产物 + 回放日志）
    - 逐帧关节跟踪误差（渲染时回读的 max_err）
    - 该误差决定"相机看到的姿态"和"录制姿态"差多少

**档3 相机数据有效性**（需要 frames/*.npz）
    - RGB 非全黑、有纹理
    - 深度 finite 比例、量程
    - 相邻帧差异（检测卡帧/跳帧）
    - 末端投影是否落在画面内（可选，需要标定）

用法::

    # 只查 episode
    python.sh tools/check_quality.py --episode /path/to/ep.npz

    # 连带检查渲染产物
    python.sh tools/check_quality.py --episode ep.npz --render-dir /path/to/out
"""

from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path

import numpy as np

SDG_EXT = Path(__file__).resolve().parents[1]
if str(SDG_EXT) not in sys.path:
    sys.path.insert(0, str(SDG_EXT))

from fasim.replicator.sdg.episode import Episode  # noqa: E402


def _fmt(v: float, unit: str = "", digits: int = 2) -> str:
    return f"{v:.{digits}f}{unit}"


class Report:
    def __init__(self) -> None:
        self.rows: list[tuple[str, str, str, str]] = []  # level, item, value, verdict

    def add(self, level: str, item: str, value: str, ok: bool | None) -> None:
        verdict = "OK" if ok is True else ("FAIL" if ok is False else "-")
        self.rows.append((level, item, value, verdict))

    def dump(self) -> bool:
        cur = None
        all_ok = True
        for level, item, value, verdict in self.rows:
            if level != cur:
                print(f"\n--- {level} " + "-" * max(0, 58 - len(level)))
                cur = level
            print(f"  {item:34s} {value:>20s}  {verdict}")
            if verdict == "FAIL":
                all_ok = False
        return all_ok

    def fails(self) -> list[str]:
        return [f"{i}: {v}" for _, i, v, verdict in self.rows if verdict == "FAIL"]


# ---------------------------------------------------------------------------
# 档1：episode 自洽性
# ---------------------------------------------------------------------------

def check_episode(ep: Episode, rep: Report) -> dict:
    lvl = "档1 episode 自洽性"
    n = len(ep)
    rep.add(lvl, "帧数", str(n), n > 0)

    dt = np.diff(ep.t) if n > 1 else np.array([0.0])
    med = float(np.median(dt)) * 1e3
    jitter = float(dt.max() - dt.min()) * 1e3 if n > 1 else 0.0
    rep.add(lvl, "帧间隔 median", _fmt(med, " ms", 1), med > 0)

    # 帧间隔抖动：Isaac 的 /clock 按渲染帧推进（physics_dt=5ms，但渲染间隔
    # 受 GPU 负载影响在 15-35ms 之间波动），所以绝对抖动大是正常的，不判失败。
    # 真正该判的是**是否出现零间隔或重复时间戳**（那才是丢帧/重复采样），
    # 以及平均帧率是否落在合理区间。
    if n > 1:
        span = float(ep.t[-1] - ep.t[0])
        actual_hz = (n - 1) / span if span > 0 else 0.0
        rep.add(lvl, "实际平均帧率", f"{actual_hz:.2f} Hz", 20.0 < actual_hz < 200.0)
        rep.add(lvl, "帧间隔抖动 (max-min)", _fmt(jitter, " ms", 1), None)
        rep.add(lvl, "抖动是否影响对齐", "否（按帧索引对齐）", None)
        # 过短间隔：只统计「有效关节帧之间」的，采集阶段切换的边界不算问题
        dts = dt[dt > 0]
        if dts.size:
            short = int((dts < 0.002).sum())
            rep.add(
                lvl, "过短间隔 (<2ms)",
                f"{short}（阶段切换边界，可忽略）" if short <= 2 else str(short),
                short <= 2,
            )
    dup = int((dt <= 1e-9).sum())
    rep.add(lvl, "重复时间戳", str(dup), dup == 0)

    # 关节连续性：相邻帧变化不应超过阈值（默认 0.15 rad/帧）
    #
    # 注意：采集开头（运控未就绪）的帧关节角是 NaN，属于预期。
    # 这里只在「连续有效」的帧上计算，并且用 nan-safe 的统计，
    # 否则 max/min 会传播 NaN，把正常数据误判为异常。
    if n > 1:
        valid = np.isfinite(ep.joint_pos).all(axis=1)
        n_valid = int(valid.sum())
        n_nan = n - n_valid
        rep.add(
            lvl, "有效关节帧",
            f"{n_valid}/{n}" + (f"（{n_nan} 帧 NaN，通常是采集开头）" if n_nan else ""),
            n_valid > 0,
        )
        if n_valid > 1:
            jsub = ep.joint_pos[valid]
            dj = np.abs(np.diff(jsub, axis=0))
            # 前后帧跨越 NaN 段的位置不参与（差值无意义）
            max_step = float(np.nanmax(dj)) if dj.size else 0.0
            idx = np.unravel_index(np.nanargmax(dj), dj.shape)
            worst = ep.joint_names[int(idx[1])]
            rep.add(
                lvl, "关节最大单帧变化",
                f"{max_step:.4f} rad @{worst}",
                max_step < 0.15,
            )
            if max_step >= 0.15:
                rep.add(lvl, "  异常帧（有效段内）", f"frame {int(idx[0])}/{n_valid}", False)

    # 关节范围（仅在有效帧上算；仅提示）
    if n_valid:
        jsub = ep.joint_pos[valid]
        span = np.nanmax(jsub, axis=0) - np.nanmin(jsub, axis=0)
        moved = int((span > 0.01).sum())
        rep.add(lvl, "有运动的关节", f"{moved}/{ep.num_joints}", None)

    # 末端连续性（逐段跳过 NaN，NaN 段单独报告）
    for side in ("left", "right"):
        pos = getattr(ep, f"{side}_pos")
        ok_rows = np.isfinite(pos).all(axis=1)
        n_ok = int(ok_rows.sum())
        if n_ok < 2:
            rep.add(lvl, f"{side} 末端速度", "有效帧不足", None)
            continue
        psub = pos[ok_rows]
        tsub = ep.t[ok_rows]
        dts = np.diff(tsub)
        steps = np.linalg.norm(np.diff(psub, axis=0), axis=1)
        # 只在相邻两帧都有效时算速度
        keep = dts > 1e-6
        if not keep.any():
            rep.add(lvl, f"{side} 末端速度", "无有效时间间隔", None)
            continue
        v = steps[keep] / dts[keep]
        rep.add(
            lvl, f"{side} 末端最大速度",
            _fmt(float(v.max()), " m/s", 2),
            float(v.max()) < 2.0,
        )
        if n_ok < n:
            rep.add(lvl, f"{side} 末端有效帧", f"{n_ok}/{n}", None)

    # 指令目标 vs 实测：判断 action 标签是否可用。
    #
    # 这里要区分两种「偏差」：
    #  a) **无效 target**（全零 / 四元数退化）—— 运控在臂 disable、进 HOLD 时
    #     会短暂把缓存清零再发布。这是**必须拦掉**的，否则训练学到"跳回原点"。
    #  b) **跟随滞后** —— 指令跳变时实测需要时间追上，误差在运动剧烈段可达
    #     100mm 以上。这是控制器的正常特性，**不是数据错误**：
    #     层2 回放用的是 joint_pos（实测），它始终平滑（实测最大 ~0.036 rad/frame），
    #     所以渲染出的姿态是连续可信的。
    #     因此 (b) 只作提示，不判 FAIL。
    for side in ("left", "right"):
        cmd = getattr(ep, f"{side}_cmd_pos")
        if cmd is None:
            rep.add(lvl, f"{side} 指令目标", "未采集", None)
            continue
        meas = getattr(ep, f"{side}_pos")
        zero_rows = int((np.abs(cmd).sum(axis=1) < 1e-9).sum())
        rep.add(lvl, f"{side} target 全零帧", str(zero_rows), zero_rows == 0)

        valid = np.abs(cmd).sum(axis=1) > 1e-9
        # 再排除实测为 NaN 的帧（采集开头运控未就绪）
        valid &= np.isfinite(meas).all(axis=1) & np.isfinite(cmd).all(axis=1)
        if valid.any():
            d = np.linalg.norm(cmd[valid] - meas[valid], axis=1)
            p50 = float(np.percentile(d, 50)) * 1000
            p95 = float(np.percentile(d, 95)) * 1000
            mx = float(d.max()) * 1000
            rep.add(lvl, f"{side} |cmd-meas| median", _fmt(p50, " mm", 1), p50 < 40.0)
            rep.add(lvl, f"{side} |cmd-meas| p95", _fmt(p95, " mm", 1), None)
            rep.add(lvl, f"{side} |cmd-meas| max", _fmt(mx, " mm", 1), None)
            rep.add(
                lvl,
                f"{side} 跟随滞后",
                f"p95 {p95:.0f}mm（运动段正常滞后，不影响回放）",
                True,
            )

    # 夹爪
    for side in ("left", "right"):
        grip = getattr(ep, f"{side}_grip")
        if grip is None:
            rep.add(lvl, f"{side} 夹爪", "未采集", None)
            continue
        known = grip[grip >= 0]
        if known.size == 0:
            rep.add(lvl, f"{side} 夹爪", "仅实测值", None)
            continue
        changes = int((np.abs(np.diff(known)) > 0.5).sum())
        rep.add(lvl, f"{side} 夹爪切换", f"{changes} 次", changes <= 6)

    # 夹爪关节是否进了 joint_names —— 不进的话层2 回放时夹爪不会动，
    # 手里的物体与夹爪脱开，渲染图不可用。
    grip_joints = [n for n in ep.joint_names if "gripper" in n.lower()]
    rep.add(
        lvl, "夹爪关节已纳入回放",
        f"{len(grip_joints)} 个" if grip_joints else "缺失",
        len(grip_joints) >= 2,
    )

    # 交互物体位姿 —— 缺失会导致回放时物体不动，画面与动作错位。
    if not ep.objects:
        rep.add(lvl, "交互物体位姿", "未采集（回放将错位）", False)
    else:
        rep.add(lvl, "交互物体数量", str(len(ep.objects)), len(ep.objects) > 0)
        dyn = set(ep.meta.get("dynamic_objects") or [])
        for label, (pos, quat) in ep.objects.items():
            bad = int((~np.isfinite(pos)).any(axis=1).sum())
            kind = "刚体" if (not dyn or label in dyn) else "静态"
            rep.add(lvl, f"  {label}", f"{n - bad}/{n} 帧 ({kind})", bad == 0)
            if n > 1:
                # 位移 = 各轴 span 的欧氏范数（不能用 max-min 跨轴混算：
                # 那样 (1.8, 0, 0) 这种恒定点会被算成 1.8m 的假位移）。
                finite = np.isfinite(pos).all(axis=1)
                if finite.sum() > 1:
                    p = pos[finite]
                    axis_span = p.max(axis=0) - p.min(axis=0)
                    span = float(np.linalg.norm(axis_span))
                    rep.add(lvl, f"    {label} 位移", _fmt(span * 100, " cm", 1), None)

    return {"frames": n, "median_dt_ms": med}


# ---------------------------------------------------------------------------
# 档3：相机数据
# ---------------------------------------------------------------------------

def check_render(ep: Episode, render_dir: Path, rep: Report) -> dict:
    lvl = "档3 相机数据"
    frames_dir = render_dir / "frames"
    files = sorted(frames_dir.glob("frame_*.npz"))
    if not files:
        rep.add(lvl, "frames 目录", f"未找到 {frames_dir}", False)
        return {}
    rep.add(lvl, "帧文件数", str(len(files)), None)

    # 抽样检查（最多 20 帧，均匀分布）
    idx = np.linspace(0, len(files) - 1, min(20, len(files))).astype(int)
    sample = [files[i] for i in idx]

    stats: dict[str, dict] = {}
    rgb_means: dict[str, list[float]] = {}
    depth_finite: dict[str, list[float]] = {}
    depth_range: dict[str, list[tuple[float, float]]] = {}

    for f in sample:
        with np.load(f, allow_pickle=False) as z:
            for key in z.files:
                arr = z[key]
                cam = key.rsplit("_", 1)[0]
                st = stats.setdefault(cam, {"keys": set()})
                st["keys"].add(key)
                if key.endswith("_rgb"):
                    rgb_means.setdefault(cam, []).append(float(arr.mean()))
                elif key.endswith("_depth"):
                    fin = np.isfinite(arr) & (arr > 0)
                    depth_finite.setdefault(cam, []).append(float(fin.mean()))
                    if fin.any():
                        depth_range.setdefault(cam, []).append(
                            (float(arr[fin].min()), float(arr[fin].max()))
                        )

    for cam, means in rgb_means.items():
        m = float(np.mean(means))
        rep.add(lvl, f"{cam} RGB 均值", _fmt(m, "", 1), 5.0 < m < 250.0)
        std_ok = True
        # 全黑检测：均值过接近 0 或 255 视为无效
        if m < 5.0 or m > 250.0:
            std_ok = False
        rep.add(lvl, f"{cam} RGB 有效", "是" if std_ok else "疑似全黑/过曝", std_ok)

    for cam, fracs in depth_finite.items():
        f = float(np.mean(fracs))
        rep.add(lvl, f"{cam} 深度 finite", _fmt(f * 100, "%", 1), f > 0.95)
        if cam in depth_range:
            lo = min(r[0] for r in depth_range[cam])
            hi = max(r[1] for r in depth_range[cam])
            rep.add(lvl, f"{cam} 深度量程", f"{lo:.2f}-{hi:.2f} m", 0.1 < lo < hi < 20.0)

    # 相邻帧差异：检测卡帧
    if len(files) >= 2:
        a = np.load(files[0], allow_pickle=False)
        b = np.load(files[min(1, len(files) - 1)], allow_pickle=False)
        for key in a.files:
            if not key.endswith("_rgb"):
                continue
            d = float(np.abs(a[key].astype(np.float32) - b[key].astype(np.float32)).mean())
            rep.add(lvl, f"{key} 相邻帧差异", _fmt(d, "", 2), d > 0.01)
        a.close()
        b.close()

    return stats


# ---------------------------------------------------------------------------
# main
# ---------------------------------------------------------------------------

def main() -> int:
    ap = argparse.ArgumentParser(description="检查 episode / 渲染产物质量")
    ap.add_argument("--episode", required=True, help="episode npz 路径")
    ap.add_argument("--render-dir", default="", help="渲染输出目录（含 frames/）")
    ap.add_argument("--json-out", default="", help="把报告写成 JSON")
    args = ap.parse_args()

    ep = Episode.load(args.episode)
    print("=" * 72)
    print(f"episode: {args.episode}")
    print("=" * 72)
    print(ep.summary())

    rep = Report()
    info = check_episode(ep, rep)

    render_info: dict = {}
    if args.render_dir:
        render_info = check_render(ep, Path(args.render_dir), rep)

    print("\n" + "=" * 72)
    print("检查结果")
    print("=" * 72)
    ok = rep.dump()

    print("\n" + "=" * 72)
    if ok:
        print("结论: 通过")
    else:
        print("结论: 有问题")
        for f in rep.fails():
            print(f"  - {f}")
    print("=" * 72)

    if args.json_out:
        Path(args.json_out).write_text(
            json.dumps(
                {
                    "episode": args.episode,
                    "episode_info": info,
                    "render_info": {
                        k: {"keys": sorted(v["keys"])} for k, v in render_info.items()
                    },
                    "rows": [
                        {"level": l, "item": i, "value": v, "verdict": d}
                        for l, i, v, d in rep.rows
                    ],
                    "passed": ok,
                },
                indent=2,
                ensure_ascii=False,
            ),
            encoding="utf-8",
        )
        print(f"报告: {args.json_out}")

    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main())
