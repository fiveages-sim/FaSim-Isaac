"""Phase 2 — replay a recorded session headlessly and render annotators.

No ROS2, no controllers: the replayer teleports the articulated robot and the
tracked rigid bodies to each recorded frame, then Replicator renders RGB-D and
camera intrinsics.
"""

from __future__ import annotations

import json
from pathlib import Path
from typing import Any

import carb
import numpy as np

from .annotators import resolve_annotators
from .types import CameraSpec, CaptureResult, RobotSpec, TaskSpec


def _log(msg: str) -> None:
    print(f"[SDG:capture] {msg}", flush=True)


def _ensure_extensions() -> None:
    from isaacsim.core.utils.extensions import enable_extension

    enable_extension("isaacsim.replicator.episode_recorder")


def _resolution_camera(camera: CameraSpec):
    import omni.replicator.core as rep

    if camera.prim_path:
        prim = camera.prim_path
        _log(f"using existing camera {camera.name}: {prim}")
        return rep.get.prim_at_path(prim)

    spec = camera.create or {}
    position = tuple(spec.get("position") or (2.0, -1.0, 1.5))
    look_at = tuple(spec.get("look_at") or (1.8, 0.2, 0.85))
    _log(f"creating camera {camera.name} at {position} look_at {look_at}")
    return rep.create.camera(position=position, look_at=look_at)


def _write_npz(out_dir: Path, frame_idx: int, payload: dict[str, Any]) -> None:
    out_dir.mkdir(parents=True, exist_ok=True)
    np.savez_compressed(out_dir / f"frame_{frame_idx:06d}.npz", **payload)


def capture_session(
    *,
    robot: RobotSpec,
    task: TaskSpec,
    hdf5_path: Path,
    output_dir: Path | None = None,
    episode_index: int = 0,
    render_interval: int = 1,
    write_raw_npz: bool = True,
) -> CaptureResult:
    """Replay ``hdf5_path`` and write annotator output per frame.

    Args:
        robot: robot prim facts (used for EE pose readout).
        task: task config (cameras, output path).
        hdf5_path: session produced by :func:`record_episode`.
        output_dir: override ``task.output_dir``.
        episode_index: which episode inside the session to replay.
        render_interval: replay every Nth recorded frame.
        write_raw_npz: dump each frame as ``frame_XXXXXX.npz``.

    Returns:
        :class:`CaptureResult` summary.
    """
    _ensure_extensions()

    import omni.replicator.core as rep
    import omni.usd
    from isaacsim.replicator.episode_recorder import EpisodeReplayer, ReplayPolicy

    out = Path(output_dir or task.output_dir)
    out.mkdir(parents=True, exist_ok=True)

    warnings: list[str] = []
    if not task.cameras:
        raise ValueError("task has no cameras configured")

    rep.orchestrator.set_capture_on_play(False)
    carb.settings.get_settings().set("rtx/post/dlss/execMode", 2)

    # --- attach cameras + writers -------------------------------------------------
    products: list[tuple[CameraSpec, Any, Any, list[str]]] = []
    for cam in task.cameras:
        annotators = list(resolve_annotators(cam.effective_annotators()))
        cam_node = _resolution_camera(cam)
        rp = rep.create.render_product(cam_node, cam.resolution)
        product_dir = out / "annotators" / cam.name
        product_dir.mkdir(parents=True, exist_ok=True)
        writer = rep.WriterRegistry.get("BasicWriter")
        writer.initialize(output_dir=str(product_dir), **{a: True for a in annotators})
        writer.attach([rp])
        products.append((cam, rp, writer, annotators))
        _log(f"attached {cam.name} {cam.resolution} annotators={annotators}")

    # --- replay -------------------------------------------------------------------
    replayer = EpisodeReplayer(str(hdf5_path), policy=ReplayPolicy(strictness="best_effort"))
    frames_written = 0
    manifest: list[dict[str, Any]] = []

    def _post_frame(idx: int) -> None:
        nonlocal frames_written
        rep.orchestrator.step(delta_time=0.0, pause_timeline=False)
        if write_raw_npz:
            payload: dict[str, Any] = {"frame_index": np.int64(idx)}
            payload["timestamp"] = np.float64(idx / max(task.capture_fps, 1))
            _write_npz(out / "frames", idx, payload)
        frames_written += 1
        manifest.append({"frame": idx, "t": idx / max(task.capture_fps, 1)})

    try:
        episodes = replayer.list_episodes()
        _log(f"session episodes: {episodes}")
        if not episodes:
            raise RuntimeError(f"no episodes in {hdf5_path}")
        if episode_index >= len(episodes):
            raise IndexError(f"episode_index {episode_index} >= {len(episodes)}")
        replayer.replay_episode(
            episode_index,
            render_interval=render_interval,
            post_frame_hook=_post_frame,
            app_update_per_frame=True,
        )
    finally:
        replayer.close()
        for _cam, _rp, writer, _ann in products:
            try:
                writer.detach()
            except Exception as exc:  # pragma: no cover - teardown best effort
                warnings.append(f"writer detach failed: {exc}")

    (out / "manifest.json").write_text(
        json.dumps(
            {
                "robot": robot.robot_key,
                "task": task.task_key,
                "session": str(hdf5_path),
                "episode": episode_index,
                "fps": task.capture_fps,
                "cameras": [c.name for c in task.cameras],
                "frames": manifest,
            },
            indent=2,
        ),
        encoding="utf-8",
    )

    _log(f"captured {frames_written} frames -> {out}")
    return CaptureResult(
        output_dir=out,
        frames_written=frames_written,
        cameras=[c.name for c in task.cameras],
        warnings=warnings,
    )
