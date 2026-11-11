"""快速验证回放：单相机、低分辨率、全程抽样，导出图片 + 拼图。

用途：**不看视窗**也能判断「按 episode 的关节角回放，机器人姿态与物体
是否真的在动」。适合在正式全量渲染前先花十几秒确认数据/流程没问题。

设计要点
========
- **直接复用 ``renderer.render_episode``**，不另写一套物理/物体逻辑。
  否则验证的东西和真正渲染的东西不是同一份代码，验证就没意义了。
- 单相机、320x240、抽样 12-20 帧，十几秒跑完（全量 2600 帧要十几分钟）。
- 物体由物理自然响应（不手动摆位），与 ``render`` 完全一致。
- 抽样区间从 0 开始覆盖全程，因此物理会从录制起点演化，物体状态正确。

用法::

    /home/fiveages/isaacsim/python.sh tools/verify_replay.py \
        --episode ~/fa_data/pick_10.npz --out /tmp/verify --samples 12

产物::

    /tmp/verify/sample_0000_t00.00.png
    ...
    /tmp/verify/contact_sheet.png     所有样本拼成一张
    /tmp/verify/report.json           每帧的关节误差 / 物体偏差 / 像素统计
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

from fasim.replicator.sdg.config_io import (  # noqa: E402
    CONFIG_ROOT,
    load_robot_config,
    load_task_config,
)
from fasim.replicator.sdg.episode import Episode  # noqa: E402


def _log(msg: str) -> None:
    print(f"[verify] {msg}", flush=True)


def main() -> int:
    ap = argparse.ArgumentParser(description="快速验证回放（单相机抽样截图）")
    ap.add_argument("--episode", required=True)
    ap.add_argument("--robot", default="", help="机器人 key（留空=自动选唯一配置）")
    ap.add_argument("--task", default="", help="任务 key（留空=自动选唯一配置）")
    ap.add_argument(
        "--camera",
        default="",
        help="用哪个相机出图；留空=任务配置里的第一个",
    )
    ap.add_argument("--out", default="/tmp/verify")
    ap.add_argument("--samples", type=int, default=12)
    ap.add_argument(
        "--end",
        type=int,
        default=0,
        help="只回放到第 N 帧（0=全程）。抓取发生在 ~1100 帧，调试接触时可截断",
    )
    ap.add_argument("--width", type=int, default=320)
    ap.add_argument("--height", type=int, default=240)
    ap.add_argument(
        "--stride",
        type=int,
        default=0,
        help="抽样间隔（帧）。0 = 用 --samples 在全程均匀抽样",
    )
    ap.add_argument("--config-root", default=None)
    ap.add_argument(
        "--no-lock-base",
        dest="lock_base",
        action="store_false",
        default=True,
        help="不锁底盘（默认锁，与录制时的 BASE_LOCK 一致）",
    )
    ap.add_argument(
        "--steps-per-frame",
        type=int,
        default=0,
        help="每帧物理步进次数。默认 0 = 按录制帧间隔自动对齐（推荐）；"
        "手动指定会让回放速度偏离录制速度，物理抓取可能失败",
    )
    args = ap.parse_args()

    out = Path(args.out).expanduser()
    out.mkdir(parents=True, exist_ok=True)

    root = Path(args.config_root) if args.config_root else CONFIG_ROOT
    robot_cfg = load_robot_config(args.robot, root)
    task_cfg = load_task_config(args.robot, args.task, root)
    ep = Episode.load(args.episode)
    n = len(ep)
    _log(f"episode: {n} 帧, {ep.num_joints} 关节")

    # 只保留一个相机，降低算力
    cams = list(task_cfg.cameras)
    if not cams:
        _log("ERROR 任务配置里没有相机")
        return 1
    cam = cams[0]
    if args.camera:
        picked = [c for c in cams if c.name == args.camera]
        if not picked:
            _log(f"ERROR 找不到相机 {args.camera}，可选: {[c.name for c in cams]}")
            return 1
        cam = picked[0]
    from dataclasses import replace

    cam = replace(cam, resolution=(args.width, args.height))
    task_cfg = replace(task_cfg, cameras=(cam,), output_dir=out)
    _log(f"相机: {cam.name} {args.width}x{args.height} @ {cam.prim_path}")

    # 抽样帧号
    if args.stride > 0:
        frames = list(range(0, n, args.stride))
    else:
        k = max(2, args.samples)
        frames = np.linspace(0, n - 1, k).astype(int).tolist()
    n_end = args.end if args.end > 0 else n
    n_end = min(n_end, n)
    frames = [f for f in frames if f < n_end]
    _log(f"抽样 {len(frames)} 帧: {frames[:8]}{' ...' if len(frames) > 8 else ''}")

    from isaacsim import SimulationApp

    app = SimulationApp(launch_config={
        "headless": True,
        "extra_args": [
            "--/app/omni.graph.scriptnode/opt_in=true",
            "--/app/omni.graph.scriptnode/enable_opt_in=false",
        ],
    })

    try:
        import cv2

        from fasim.replicator.sdg import renderer as R

        # render_episode 全程推进物理（物体需要连续演化），
        # 但只在 render_indices 指定的帧真正出图 —— 省掉绝大部分渲染开销。
        keep = set(int(x) for x in frames)
        result = R.render_episode(
            episode_path=Path(args.episode),
            robot_config=robot_cfg,
            task_config=task_cfg,
            output_dir=out,
            app=app,
            start=0,
            end=n_end,
            steps_per_frame=args.steps_per_frame,
            verify_joints=True,
            render_indices=keep,
            lock_base=args.lock_base,
        )

        _log("")
        _log(f"处理 {result.frames_written}/{n} 帧（只渲染抽样帧）")
        _log(f"关节 max_err = {result.joint_max_error*1000:.2f} mrad")
        p95 = result.extras.get("joint_error_p95")
        if p95 is not None:
            _log(f"关节 p95_err = {p95*1000:.2f} mrad")

        # 收集已落盘的帧文件 → 拼图
        saved: list[tuple[float, np.ndarray]] = []
        for f in sorted((out / "frames").glob("frame_*.npz")):
            try:
                idx = int(f.stem.split("_")[1])
            except Exception:  # noqa: BLE001
                continue
            if idx >= n:
                continue
            with np.load(f, allow_pickle=False) as z:
                key = next((k for k in z.files if k.endswith("_rgb")), None)
                if key is None:
                    continue
                arr = z[key]
                img = arr[..., :3].copy() if arr.shape[-1] >= 3 else None
            if img is None:
                continue
            saved.append((float(ep.t[idx]), img))

        if saved:
            saved.sort(key=lambda x: x[0])
            cols = 4
            rows = (len(saved) + cols - 1) // cols
            h, w = saved[0][1].shape[:2]
            sheet = np.zeros((rows * (h + 22), cols * w, 3), dtype=np.uint8)
            for idx, (t, img) in enumerate(saved):
                r, c = divmod(idx, cols)
                y0 = r * (h + 22)
                sheet[y0:y0 + h, c * w:c * w + w] = img.astype(np.uint8)
                cv2.putText(
                    sheet, f"t={t:.1f}s", (c * w + 4, y0 + h + 16),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.45, (255, 255, 255), 1, cv2.LINE_AA,
                )
            sheet_path = out / "contact_sheet.png"
            cv2.imwrite(str(sheet_path), cv2.cvtColor(sheet, cv2.COLOR_RGB2BGR))
            _log(f"拼图: {sheet_path}")

            base = saved[0][1].astype(np.float32)
            _log("相对第一张的像素差异:")
            for t, img in saved:
                d = float(np.abs(img.astype(np.float32) - base).mean())
                _log(f"  t={t:6.2f}s  diff={d:7.3f}")

        (out / "report.json").write_text(
            json.dumps({
                "episode": str(args.episode),
                "camera": cam.name,
                "frames_total": n,
                "frames_rendered": result.frames_written,
                "driven_joints": len(result.dof_names) - len(result.skipped_dofs),
                "skipped_dofs": result.skipped_dofs,
                "missing_joints": result.missing_joints,
                "joint_max_error_mrad": result.joint_max_error * 1000,
                "joint_p95_error_mrad": (p95 * 1000) if p95 is not None else None,
                "wall_seconds": result.wall_seconds,
                "object_deviation": result.extras.get("object_deviation"),
            }, indent=2, ensure_ascii=False),
            encoding="utf-8",
        )
        _log(f"报告: {out / 'report.json'}")
        return 0
    finally:
        app.close()


if __name__ == "__main__":
    sys.exit(main())
