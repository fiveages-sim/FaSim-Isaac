"""DOF-based replay: drive joint targets instead of teleporting link poses.

Why not use ``EpisodeReplayer``
-------------------------------
``ArticulationRecordable`` stores **47 link world poses** (not DOF angles) and
``EpisodeReplayer`` writes them back with ``set_world_poses``. For an
articulated robot that is physically illegal: one call overwrites every link at
once, so PhysX sees joint constraints violated and (a) invalidates the tensor
view (``Simulation view object is invalidated ... getDofVelocityTargets``) and
(b) lets rigidly-attached parts like the gripper rip off.

The correct loop for articulation is DOF-based::

    record:  q[t] = articulation.get_dof_positions()
    replay:  articulation.set_dof_position_targets(q[t])
             app.update()          # let the PD drives track the target

This module records DOF angles into a small .npz per episode and replays them
through the normal drive pipeline, so physics stays consistent.

Record (Script Editor, while OCS2 drives the robot)::

    from scripts.dof_replay import arm_dof_record, stop_dof_record
    arm_dof_record()
    # ... run motion-generation ...
    stop_dof_record()

Replay::

    from scripts.dof_replay import replay_dof
    replay_dof("/home/fiveages/fasim_sdg/.../dof_episode_xxx.npz")
"""

from __future__ import annotations

import sys
import time
from pathlib import Path

SDG_EXT = Path("/home/fiveages/FaSim-Isaac/exts/fasim.replicator.sdg")
if str(SDG_EXT) not in sys.path:
    sys.path.insert(0, str(SDG_EXT))

_STATE: dict = {"recording": False, "rows": [], "sub": None, "path": None, "mask": None}


def _replay_dof_mask(art) -> tuple["np.ndarray", list[str]]:
    """Indices of DOFs that should be replayed (body/arms/grippers).

    The G1D articulation also owns the drive wheels and the four caster
    swivel/wheel joints. Those are *not* manipulation DOFs: forcing them with
    ``set_dof_position_targets`` turns the wheels, which drags the whole chassis
    around during replay (the "chassis rotates by itself" symptom). They are also
    physically meaningless to replay — the base pose is not part of the
    trajectory we recorded.

    Returns:
        ``(mask, skipped_names)`` where ``mask`` is a bool array over DOFs.
    """
    import numpy as np

    names = list(art.dof_names)
    skip_tokens = ("wheel", "caster")
    mask = np.array(
        [not any(tok in n.lower() for tok in skip_tokens) for n in names],
        dtype=bool,
    )
    skipped = [n for n, keep in zip(names, mask) if not keep]
    return mask, skipped


def _config(robot_key: str, task_key: str):
    from fasim.replicator.sdg.config_io import load_robot_config, load_task_config

    return load_robot_config(robot_key), load_task_config(robot_key, task_key)


def arm_dof_record(
    robot_key: str = "unitree_g1d",
    task_key: str = "sinopec_test_tube",
    *,
    sample_hz: float = 30.0,
    output_hint_dir: str | None = None,
) -> dict:
    """Start sampling DOF positions every Kit tick (throttled to ``sample_hz``).

    Non-blocking: subscribes to ``omni.kit.app.GLOBAL_EVENT_UPDATE``.
    """
    stop_dof_record()
    import carb.eventdispatcher
    import omni.kit.app
    from isaacsim.core.experimental.prims import Articulation

    robot, task = _config(robot_key, task_key)
    art = Articulation(robot.articulation_prim)
    n = int(art.num_dofs)

    stamp = time.strftime("%Y%m%dT%H%M%SZ", time.gmtime())
    out_dir = Path(output_hint_dir) if output_hint_dir else Path(task.output_dir)
    out_dir.mkdir(parents=True, exist_ok=True)
    path = out_dir / f"dof_episode_{stamp}.npz"

    period = 1.0 / max(sample_hz, 1.0)
    last = {"t": 0.0}
    rows: list = []

    def _on_update(_evt) -> None:
        if not _STATE["recording"]:
            return
        now = time.monotonic()
        if now - last["t"] < period:
            return
        last["t"] = now
        try:
            q = art.get_dof_positions().numpy().reshape(-1)[:n]
            rows.append(q.astype("float32"))
        except Exception:
            # physics view can be transient right after Play; skip this tick
            pass

    _STATE.update(recording=True, rows=rows, path=str(path), n=n)
    _STATE["sub"] = carb.eventdispatcher.get_eventdispatcher().observe_event(
        event_name=omni.kit.app.GLOBAL_EVENT_UPDATE,
        on_event=_on_update,
        observer_name="dof_replay.record",
    )
    print(f"[dof] RECORDING dofs={n} @{sample_hz:.0f}Hz -> {path}")
    print("[dof] run motion-generation, then call stop_dof_record()")
    return {"path": str(path), "dofs": n}


def stop_dof_record() -> dict:
    """Stop sampling and write the .npz."""
    import numpy as np

    rows = _STATE.get("rows") or []
    path = _STATE.get("path")
    sub = _STATE.get("sub")
    if sub is not None:
        try:
            sub.unsubscribe() if hasattr(sub, "unsubscribe") else None
        except Exception:
            pass
    _STATE.update(recording=False, sub=None)

    if not rows or not path:
        print("[dof] nothing recorded")
        return {}

    arr = np.asarray(rows, dtype="float32")
    Path(path).parent.mkdir(parents=True, exist_ok=True)
    np.savez_compressed(path, dof_positions=arr)
    print(f"[dof] DONE frames={arr.shape[0]} dofs={arr.shape[1]} -> {path}")
    print(f"[dof] replay: replay_dof({path!r})")
    _STATE.update(rows=[], path=None)
    return {"path": str(path), "frames": int(arr.shape[0])}


def replay_dof(
    npz_path: str,
    *,
    robot_key: str = "unitree_g1d",
    task_key: str = "sinopec_test_tube",
    speed: float = 1.0,
    start: int = 0,
    end: int | None = None,
) -> dict:
    """Replay DOF angles by driving joint position targets (physics stays valid).

    Args:
        npz_path: file written by :func:`stop_dof_record`.
        robot_key/task_key: config selection (for the articulation prim).
        speed: playback multiplier (1.0 = recorded rate).
        start/end: optional frame slice.

    Returns:
        ``{"frames": N, "dofs": M}``.
    """
    import numpy as np
    import omni.kit.app
    from isaacsim.core.experimental.prims import Articulation

    robot, task = _config(robot_key, task_key)

    data = np.load(npz_path)
    track = data["dof_positions"]
    total = int(track.shape[0])
    lo = max(0, int(start))
    hi = total if end is None else min(total, int(end))
    if hi <= lo:
        raise ValueError(f"empty frame range [{lo}, {hi}) for {total} frames")

    art = Articulation(robot.articulation_prim)
    n = int(art.num_dofs)
    if track.shape[1] < n:
        raise ValueError(f"recorded {track.shape[1]} dofs but robot has {n}")

    mask, skipped = _replay_dof_mask(art)
    print(f"[dof] replaying {int(mask.sum())}/{n} dofs (skipping {len(skipped)}: {skipped})")

    # Timeline must be playing: set_dof_position_targets needs the physics step.
    import omni.timeline

    tl = omni.timeline.get_timeline_interface()
    if tl.is_playing():
        print(
            "[dof] WARNING: timeline is PLAYING. If a controller (OCS2) is still "
            "active it will keep writing joint targets and fight this replay "
            "(symptom: violent shaking). Put the FSM in HOLD "
            "(ros2 topic pub --once /fsm_command std_msgs/msg/Int32 '{data: 2}') "
            "before replaying."
        )
    else:
        tl.play()
        print("[dof] timeline PLAYING")

    app = omni.kit.app.get_app()

    # Physics initializes on the first ticks after Play; reading DOF state before
    # that raises "physics tensor entity is not valid".
    for _ in range(10):
        app.update()

    period = (1.0 / max(task.capture_fps, 1)) / max(speed, 1e-3)

    # Hold skipped DOFs (wheels/casters) at their CURRENT position so they neither
    # spin nor get dragged: build a full-width target buffer once, then only
    # overwrite the replayed columns per frame.
    current = art.get_dof_positions().numpy().reshape(-1)[:n].astype(np.float32)
    base_target = current.copy()

    print(f"[dof] replay {hi - lo}/{total} frames, dofs={int(mask.sum())}, speed={speed}x")
    t_next = time.monotonic()
    for i in range(lo, hi):
        target = base_target.copy()
        target[mask] = track[i][:n][mask]
        art.set_dof_position_targets(target[np.newaxis, :].astype(np.float32))
        # pump enough ticks for the drive to track before the next sample
        for _ in range(2):
            app.update()
        t_next += period
        sleep_for = t_next - time.monotonic()
        if sleep_for > 0:
            time.sleep(sleep_for)
        else:
            t_next = time.monotonic()

    print(f"[dof] FINISHED ({hi - lo} frames)")
    return {"frames": hi - lo, "dofs": int(mask.sum())}
