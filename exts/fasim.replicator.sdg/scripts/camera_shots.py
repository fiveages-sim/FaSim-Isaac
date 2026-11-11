"""把一次采集会话的相机帧拼成联络图（contact sheet），人工检查 RGB 是否正常。

为什么单独写脚本
================
采集本身跑在 Isaac 进程里，看图像却只需要读落盘的 ``frames/*.npz`` —— 不必
再开 Isaac、不必重新渲染。所以这是一个**完全离线**的小工具。

用法::

    # 默认：每个相机一张 8x6 = 48 张的拼图，覆盖整个会话
    ./scripts/sdg.sh shots ~/fasim_data/cam01

    # 只看任务段（按关节运动幅度自动切，剔除待机）
    ./scripts/sdg.sh shots ~/fasim_data/cam01 --segment

    # 自定义网格 / 指定相机 / 缩小单图
    ./scripts/sdg.sh shots ~/fasim_data/cam01 --cols 10 --rows 5 --scale 0.4
    ./scripts/sdg.sh shots ~/fasim_data/cam01 --camera head

    # 只取某个时间窗（秒，相对会话开始）
    ./scripts/sdg.sh shots ~/fasim_data/cam01 --start 5 --end 25

产物::

    <session>/contact_<camera>.png     每相机一张
    <session>/contact_index.json       每张缩略图对应的帧号/时间（便于回查）

判断要点（脚本末尾也会打印）
============================
1. **帧间必须有变化** —— 若几十张图完全一样，说明相机没被渲染或取到了冻结缓冲。
2. **画面不能全黑/全白** —— 全黑常见于相机朝向错误或曝光问题。
3. **腕部相机应随手臂移动** —— 头相机基本固定，腕部相机会大幅变化。
4. **RGB 与深度对齐** —— 同一帧的 ``*_rgb`` 与 ``*_depth`` 尺寸必须一致。
"""

from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path

import numpy as np


def _log(msg: str) -> None:
    print(f"[shots] {msg}", flush=True)


# ---------------------------------------------------------------- 读帧


def load_frame_paths(session: Path) -> list[Path]:
    frames = session / "frames"
    if not frames.is_dir():
        raise FileNotFoundError(f"没有 frames 目录: {frames}")
    fs = sorted(frames.glob("frame_*.npz"))
    if not fs:
        raise ValueError(f"没有帧文件: {frames}")
    return fs


def available_cameras(files: list[Path]) -> list[str]:
    """扫第一帧的 key，找出所有 ``<name>_rgb``。"""
    with np.load(files[0], allow_pickle=False) as z:
        keys = list(z.files)
    cams = []
    for k in keys:
        if k.endswith("_rgb"):
            cams.append(k[: -len("_rgb")])
    return sorted(cams)


def _sim_steps(files: list[Path]) -> np.ndarray:
    out = []
    for f in files:
        try:
            with np.load(f, allow_pickle=False) as z:
                out.append(int(z["sim_step"]) if "sim_step" in z.files else len(out))
        except Exception:  # noqa: BLE001
            out.append(len(out))
    return np.asarray(out, dtype=np.int64)


def activity_mask(files: list[Path]) -> np.ndarray | None:
    """按关节角速度判「任务区间」（剔除待机段）。

    和 ``trim`` 同一个思路，但这里只需要一个粗区间：把逐帧关节差分求和，
    取超过中位数的连续区间。拿不到关节就返回 ``None``。
    """
    qs = []
    for f in files:
        try:
            with np.load(f, allow_pickle=False) as z:
                if "joint_pos" not in z.files:
                    return None
                qs.append(np.asarray(z["joint_pos"], dtype=np.float64).reshape(-1))
        except Exception:  # noqa: BLE001
            return None
    if len(qs) < 3:
        return None
    q = np.stack(qs)
    speed = np.abs(np.diff(q, axis=0)).sum(axis=1)
    thr = max(float(np.median(speed)) * 2.0, 1e-4)
    active = np.concatenate([[False], speed > thr])
    if not active.any():
        return None
    idx = np.flatnonzero(active)
    lo, hi = int(idx[0]), int(idx[-1])
    if hi - lo < 5:
        return None
    mask = np.zeros(len(files), dtype=bool)
    mask[lo:hi + 1] = True
    return mask


# ---------------------------------------------------------------- 拼图


def _thumb(rgb: np.ndarray, scale: float) -> np.ndarray:
    """RGB(A) → 缩小后的 RGB uint8。用**整数步长抽样**，不需要 PIL。"""
    a = np.asarray(rgb)
    if a.ndim == 3 and a.shape[2] >= 3:
        a = a[..., :3]
    if a.dtype != np.uint8:
        a = np.clip(a, 0, 255).astype(np.uint8)
    if scale != 1.0 and scale > 0:
        step = max(1, int(round(1.0 / scale)))
        a = a[::step, ::step]
    return a


def _label_strip(width: int, text: str, height: int = 14) -> np.ndarray:
    """在缩略图上方加一条纯色条，靠亮度区分，避免依赖字体渲染。

    文字本身无法绘制（不引 PIL），所以用**亮度编码**：左侧亮度块随帧号
    线性变化，视觉上能看出"这是第几张、有没有重复"。
    """
    strip = np.zeros((height, width, 3), dtype=np.uint8)
    return strip


def make_sheet(
    files: list[Path],
    camera: str,
    indices: list[int],
    cols: int,
    rows: int,
    scale: float,
    gap: int = 2,
) -> tuple[np.ndarray | None, list[dict]]:
    """把选中的帧拼成一张网格图。"""
    tiles: list[np.ndarray] = []
    index: list[dict] = []
    for n, i in enumerate(indices[: cols * rows]):
        try:
            with np.load(files[i], allow_pickle=False) as z:
                key = f"{camera}_rgb"
                if key not in z.files:
                    continue
                t = _thumb(z[key], scale)
                step = int(z["sim_step"]) if "sim_step" in z.files else -1
        except Exception as exc:  # noqa: BLE001
            _log(f"WARN 读第 {i} 帧失败: {exc}")
            continue
        # 左上角画一个亮度标记条：位置 n 越靠后越亮，便于发现重复帧
        h, w = t.shape[:2]
        mark_w = max(2, w // 12)
        level = int(255 * (n / max(1, len(indices) - 1)))
        t = t.copy()
        t[0:4, 0:mark_w] = np.array([level, level, level], dtype=np.uint8)
        tiles.append(t)
        index.append({"tile": n, "frame_index": int(i), "sim_step": int(step)})

    if not tiles:
        return None, index

    th, tw = tiles[0].shape[:2]
    # 统一尺寸（不同相机分辨率可能不同）
    norm = []
    for t in tiles:
        if t.shape[:2] != (th, tw):
            hh, ww = min(th, t.shape[0]), min(tw, t.shape[1])
            c = np.zeros((th, tw, 3), dtype=np.uint8)
            c[:hh, :ww] = t[:hh, :ww]
            norm.append(c)
        else:
            norm.append(t)
    tiles = norm

    sheet_h = rows * th + (rows - 1) * gap
    sheet_w = cols * tw + (cols - 1) * gap
    sheet = np.full((sheet_h, sheet_w, 3), 32, dtype=np.uint8)
    for n, t in enumerate(tiles):
        r, c = divmod(n, cols)
        y = r * (th + gap)
        x = c * (tw + gap)
        sheet[y:y + th, x:x + tw] = t
    return sheet, index


def save_png(path: Path, img: np.ndarray) -> bool:
    """优先用 imageio/PIL；都没有就退化成 npz（仍可看，但要自己读）。"""
    for mod in ("imageio.v2", "imageio"):
        try:
            m = __import__(mod, fromlist=["imwrite"])
            m.imwrite(str(path), img)
            return True
        except Exception:  # noqa: BLE001
            pass
    try:
        from PIL import Image

        Image.fromarray(img).save(str(path))
        return True
    except Exception:  # noqa: BLE001
        pass
    alt = path.with_suffix(".npy")
    np.save(alt, img)
    _log(f"WARN 没有 imageio/PIL，已存为 {alt}（用 np.load 查看）")
    return False


# ---------------------------------------------------------------- 健全性


def check_frames(files: list[Path], camera: str) -> dict:
    """逐帧统计 RGB/深度，给出"是否正常"的量化判据。"""
    means: list[float] = []
    stds: list[float] = []
    uniq: list[int] = []
    depth_stats: list[tuple[float, float, int]] = []
    size_mismatch = 0
    for f in files:
        try:
            with np.load(f, allow_pickle=False) as z:
                rk = f"{camera}_rgb"
                if rk not in z.files:
                    continue
                a = np.asarray(z[rk])
                rgb = a[..., :3] if a.ndim == 3 else a
                means.append(float(rgb.mean()))
                stds.append(float(rgb.std()))
                # 唯一色数（粗估）—— 全黑/全白会非常小
                flat = rgb.reshape(-1, 3)[:: max(1, rgb.size // 3 // 2000)]
                uniq.append(int(len(np.unique(flat, axis=0))))
                dk = f"{camera}_depth"
                if dk in z.files:
                    d = np.asarray(z[dk])
                    if d.shape[:2] != rgb.shape[:2]:
                        size_mismatch += 1
                    good = np.isfinite(d) & (d > 0)
                    depth_stats.append((
                        float(d[good].min()) if good.any() else float("nan"),
                        float(d[good].max()) if good.any() else float("nan"),
                        int((~good).sum()),
                    ))
        except Exception:  # noqa: BLE001
            continue

    out: dict = {
        "camera": camera,
        "frames": len(means),
        "rgb_mean": round(float(np.mean(means)), 1) if means else None,
        "rgb_mean_range": [round(min(means), 1), round(max(means), 1)] if means else None,
        "rgb_std_mean": round(float(np.mean(stds)), 1) if stds else None,
        "uniq_colors_median": int(np.median(uniq)) if uniq else None,
        "rgb_depth_size_mismatch": size_mismatch,
    }
    if depth_stats:
        arr = np.array([[a, b, c] for a, b, c in depth_stats], dtype=np.float64)
        out["depth_min"] = round(float(np.nanmin(arr[:, 0])), 3)
        out["depth_max"] = round(float(np.nanmax(arr[:, 1])), 3)
        out["depth_invalid_frac"] = round(
            float(np.nanmean(arr[:, 2]) / max(1, arr[:, 2].max() + 1)), 4
        )
    # 结论
    warns = []
    if out["rgb_mean"] is not None:
        if out["rgb_mean"] < 1.0:
            warns.append("画面几乎全黑")
        elif out["rgb_mean"] > 254.0:
            warns.append("画面几乎全白")
        if (out["uniq_colors_median"] or 0) < 5:
            warns.append("色彩几乎没有变化（可能取到冻结缓冲）")
        rng = out["rgb_mean_range"]
        if rng and abs(rng[1] - rng[0]) < 0.5:
            warns.append("所有帧亮度几乎相同（画面可能冻结）")
    if size_mismatch:
        warns.append(f"RGB 与深度尺寸不一致（{size_mismatch} 帧）")
    out["warnings"] = warns
    return out


# ---------------------------------------------------------------- 入口


def main(argv: list[str] | None = None) -> int:
    ap = argparse.ArgumentParser(
        prog="shots",
        description="把采集会话的相机帧拼成联络图，人工检查 RGB 是否正常",
    )
    ap.add_argument("session", help="采集会话目录（含 frames/）")
    ap.add_argument("--camera", default="", help="只处理该相机（逗号分隔）；留空=全部")
    ap.add_argument("--cols", type=int, default=8, help="网格列数（默认 8）")
    ap.add_argument("--rows", type=int, default=6, help="网格行数（默认 6）")
    ap.add_argument("--scale", type=float, default=0.5,
                    help="单图缩放（默认 0.5；1.0=原尺寸）")
    ap.add_argument("--segment", action="store_true",
                    help="只取任务段（按关节运动幅度自动切，剔除待机）")
    ap.add_argument("--start", type=float, default=0.0, help="起始时间（秒）")
    ap.add_argument("--end", type=float, default=0.0, help="结束时间（秒）；0=到末尾")
    ap.add_argument("--gap", type=int, default=2, help="间距像素")
    args = ap.parse_args(argv)

    session = Path(args.session).expanduser().resolve()
    try:
        files = load_frame_paths(session)
    except Exception as exc:  # noqa: BLE001
        _log(f"ERROR {exc}")
        return 1

    cams = available_cameras(files)
    if not cams:
        _log("ERROR 帧里没有 <camera>_rgb 字段 —— 采集时没选相机？"
             "用：./scripts/sdg.sh capture --what camera,joint,ee,object")
        return 1
    want = [c.strip() for c in str(args.camera).split(",") if c.strip()]
    if want:
        cams = [c for c in cams if c in want] or cams

    # 时间轴（来自 session.json 的 physics_dt × every，退化为帧序）
    dt = 0.05
    sj = session / "session.json"
    if sj.is_file():
        try:
            meta = json.loads(sj.read_text(encoding="utf-8"))
            pdt = float(meta.get("physics_dt", 0) or 0)
            every = int(meta.get("every", 1) or 1)
            if pdt > 0:
                dt = pdt * every
        except Exception:  # noqa: BLE001
            pass
    steps = _sim_steps(files)
    n = len(files)

    idx = list(range(n))
    if args.segment:
        mask = activity_mask(files)
        if mask is not None:
            idx = [i for i in idx if mask[i]]
            _log(f"任务段: {len(idx)}/{n} 帧（按关节运动幅度自动切）")
        else:
            _log("WARN 无法判定任务段（缺关节数据），用全部帧")
    if args.start > 0 or args.end > 0:
        lo = int(args.start / dt) if args.start > 0 else 0
        hi = int(args.end / dt) if args.end > 0 else n
        idx = [i for i in idx if lo <= i < hi]
        _log(f"时间窗 {args.start}~{args.end or 'end'}s → {len(idx)} 帧")

    if not idx:
        _log("ERROR 选不到帧（检查 --start/--end/--segment）")
        return 1

    # 均匀抽样到 cols*rows 张
    need = args.cols * args.rows
    if len(idx) > need:
        pick = np.linspace(0, len(idx) - 1, need).round().astype(int)
        sel = [idx[int(i)] for i in pick]
    else:
        sel = idx

    _log(f"会话: {session}")
    _log(f"帧数: {n}   相机: {cams}   网格: {args.rows}x{args.cols} = {need}")
    _log(f"取样: {len(sel)} 帧（等间隔覆盖）")

    reports = []
    ok_all = True
    for cam in cams:
        sheet, index = make_sheet(
            files, cam, sel, args.cols, args.rows, args.scale, args.gap
        )
        rep = check_frames(files, cam)
        reports.append(rep)
        if rep.get("warnings"):
            ok_all = False
        if sheet is None:
            _log(f"WARN 相机 {cam} 没有可用图像")
            continue
        out_png = session / f"contact_{cam}.png"
        saved = save_png(out_png, sheet)
        _idx = session / f"contact_{cam}_index.json"
        _idx.write_text(
            json.dumps(
                {"camera": cam, "session": str(session),
                 "grid": [args.rows, args.cols], "tiles": index},
                indent=2, ensure_ascii=False,
            ),
            encoding="utf-8",
        )
        _log(f"  {cam}: {sheet.shape[1]}x{sheet.shape[0]} → {out_png if saved else out_png.with_suffix('.npy')}")

    # ---- 结论 ----
    print()
    print("=" * 68)
    print("相机数据体检")
    print("=" * 68)
    for r in reports:
        print(f"\n[{r['camera']}]  帧数={r['frames']}")
        print(f"  RGB 均值 {r['rgb_mean']}  范围 {r['rgb_mean_range']}"
              f"  std={r['rgb_std_mean']}  唯一色(中位)={r['uniq_colors_median']}")
        if "depth_min" in r:
            print(f"  深度 {r['depth_min']}~{r['depth_max']} m  "
                  f"无效占比 {r['depth_invalid_frac']*100:.2f}%")
        if r["rgb_depth_size_mismatch"]:
            print(f"  ⚠ RGB/深度尺寸不一致: {r['rgb_depth_size_mismatch']} 帧")
        if r["warnings"]:
            for w in r["warnings"]:
                print(f"  ✗ {w}")
        else:
            print("  ✅ 量化检查通过")
    print()
    print("=" * 68)
    print("请人工检查拼图：")
    print("  1) 帧间应有明显变化（亮度条从左到右变亮只是标记，画面内容要变）")
    print("  2) 不能全黑/全白/纯色")
    print("  3) 头相机基本固定；腕部相机应随手臂大幅移动")
    print("  4) 抓取段应能看到试管、夹爪、架子")
    if ok_all:
        print("\n结论: 量化检查全部通过 ✅")
    else:
        print("\n结论: 有警告，请看上面的 ✗ 项")
    print("=" * 68)
    return 0 if ok_all else 2


if __name__ == "__main__":
    sys.exit(main())
