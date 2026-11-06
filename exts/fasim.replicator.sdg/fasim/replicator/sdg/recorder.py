"""Phase 1 — record articulation + rigid-body state to HDF5.

Deliberately depends on **nothing but** the USD stage and (optionally) a live
``/joint_states`` stream. That keeps the trajectory source pluggable:

    OCS2 ros2_control  ->  /joint_states  ->  here
    VR teleop          ->  /joint_states  ->  here
    scripted motion    ->  /joint_states  ->  here

The recorder never imports ros2_control, OCS2, or lerobot code.
"""

from __future__ import annotations

import os
import time
from pathlib import Path
from typing import Any

import carb
from .types import RecordResult, RobotSpec, TaskSpec


def _log(msg: str) -> None:
    print(f"[SDG:record] {msg}", flush=True)


def _ensure_extension() -> None:
    from isaacsim.core.utils.extensions import enable_extension

    enable_extension("isaacsim.replicator.episode_recorder")


def _apply_variants(prim_path: str, selection: dict[str, str]) -> None:
    import omni.usd

    if not selection:
        return
    stage = omni.usd.get_context().get_stage()
    prim = stage.GetPrimAtPath(prim_path)
    if not prim or not prim.IsValid():
        raise RuntimeError(f"variant target prim not found: {prim_path}")
    vsets = prim.GetVariantSets()
    for name, value in selection.items():
        if not vsets.HasVariantSet(name):
            carb.log_warn(f"[SDG:record] variant set {name!r} absent on {prim_path}; skipping")
            continue
        vsets.GetVariantSet(name).SetVariantSelection(value)
        _log(f"variant {prim_path} {name}={value}")


def _open_stage(scene_usda: Path) -> None:
    import omni.usd

    if not scene_usda.is_file():
        raise FileNotFoundError(f"scene not found: {scene_usda}")
    ctx = omni.usd.get_context()
    ctx.open_stage(str(scene_usda))
    _log(f"opened {scene_usda}")


def record_episode(
    *,
    robot: RobotSpec,
    task: TaskSpec,
    app: Any,
    output_dir: Path | None = None,
    duration_sec: float = 30.0,
    success: bool = True,
    session_metadata: dict[str, Any] | None = None,
) -> RecordResult:
    """Record one episode of live simulation state.

    The caller is responsible for having the robot already moving (OCS2, VR
    teleop, script ...). This function only *observes*.

    Args:
        robot: robot-level prim/topic facts.
        task: task-level scene/output facts.
        app: the live ``SimulationApp`` — required because
            :class:`EpisodeRecorder` writes on physics-step events, so the app
            loop must be pumped from here (``time.sleep`` alone records 0 frames).
        output_dir: override ``task.output_dir``.
        duration_sec: wall-clock recording window.
        success: value stored in the episode metadata.
        session_metadata: extra key/values saved alongside the session.

    Returns:
        :class:`RecordResult` with the HDF5 path and frame count.
    """
    _ensure_extension()

    from isaacsim.replicator.episode_recorder import (
        ArticulationRecordable,
        EpisodeRecorder,
        RigidBodyRecordable,
        export_stage_snapshot,
    )
    from isaacsim.core.experimental.prims import Articulation

    out = Path(output_dir or task.output_dir)
    out.mkdir(parents=True, exist_ok=True)

    snapshot = None
    try:
        snapshot = export_stage_snapshot(str(out))
        _log(f"stage snapshot: {snapshot}")
    except Exception as exc:  # snapshot is best-effort
        carb.log_warn(f"[SDG:record] stage snapshot failed: {exc}")

    meta = {
        "task_description": task.task_key,
        "robot_key": robot.robot_key,
        "capture_fps": task.capture_fps,
        "physics_dt": task.physics_dt,
        "source": "live_joint_states",
        **(session_metadata or {}),
    }
    recorder = EpisodeRecorder(output_dir=str(out), session_metadata=meta)

    recorder.add(
        ArticulationRecordable(group=f"state/{robot.robot_key}", prim_path=robot.articulation_prim)
    )
    for label, prim in task.tracked_prims.items():
        recorder.add(RigidBodyRecordable(group=f"state/{label}", prim_path=prim))

    hdf5_path = recorder.open_session()
    _log(f"session opened: {hdf5_path}")

    frames = 0
    t0 = time.monotonic()
    recorder.start_episode()
    try:
        import omni.timeline

        timeline = omni.timeline.get_timeline_interface()
        if not timeline.is_playing():
            _log("WARNING: timeline is not playing; recorder will capture static state")

        # EpisodeRecorder writes on physics-step events. In a standalone/headless
        # run nothing else drives the app loop, so we must pump ``app.update()``
        # ourselves — ``time.sleep`` alone yields an empty (0-frame) session.
        period = 1.0 / max(task.capture_fps, 1)
        next_tick = time.monotonic()
        while (time.monotonic() - t0) < duration_sec:
            app.update()
            frames += 1
            next_tick += period
            sleep_for = next_tick - time.monotonic()
            if sleep_for > 0:
                time.sleep(sleep_for)
            else:
                next_tick = time.monotonic()
    finally:
        recorder.end_episode(success=success)
        recorder.close_session()

    _log(f"recorded {duration_sec:.1f}s (~{frames} @{task.capture_fps}Hz) -> {hdf5_path}")
    return RecordResult(
        hdf5_path=Path(hdf5_path),
        snapshot_path=Path(snapshot) if snapshot else None,
        frames=frames,
        duration_sec=duration_sec,
    )
