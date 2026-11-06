"""Drop-in recorder for an already-running Isaac Sim GUI session.

Paste into Isaac Sim: ``Window > Script Editor`` (Python), press Run.

CRITICAL: never block inside the Script Editor. A ``while time.sleep(...)`` loop
starves Kit's main loop and freezes the whole app (physics stops, OCS2 stalls).
Everything here is therefore *non-blocking*: :func:`arm` returns immediately and
:func:`stop` finishes the session when you are done.

Typical flow::

    # 1) arm (returns instantly)
    arm()
    # -> [recorder] ARMED. Run motion-generation now, then call stop()

    # 2) in another terminal:
    #    motion-generation --robot unitree_g1d --task-key pick_tube_full --scene lift

    # 3) back in Script Editor:
    stop()
    # -> [recorder] DONE frames=... hdf5=/path/episode_xxx.hdf5
"""

from __future__ import annotations

import sys
from pathlib import Path

SDG_EXT = Path("/home/fiveages/FaSim-Isaac/exts/fasim.replicator.sdg")
if str(SDG_EXT) not in sys.path:
    sys.path.insert(0, str(SDG_EXT))

#: module-level handle so ``stop()`` can find the armed session
_STATE: dict = {"recorder": None, "hdf5": None, "episode": None, "out": None, "frames": 0}


def arm(
    robot_key: str = "unitree_g1d",
    task_key: str = "sinopec_test_tube",
    output_dir: str | None = None,
) -> dict | None:
    """Arm the episode recorder on the live stage. Returns immediately.

    Safe to call from the Script Editor: it only creates the recorder and
    subscribes to physics post-step events, then returns.

    Args:
        robot_key/task_key: select ``configs/<robot>/`` + ``tasks/<task>.yaml``.
        output_dir: override the configured output root.

    Returns:
        ``{"hdf5":..., "episode":..., "output_dir":...}`` or ``None`` if already armed.
    """
    if _STATE["recorder"] is not None:
        print("[recorder] already armed; call stop() first")
        return None

    from isaacsim.core.utils.extensions import enable_extension

    enable_extension("isaacsim.replicator.episode_recorder")

    from isaacsim.replicator.episode_recorder import (
        ArticulationRecordable,
        EpisodeRecorder,
        RigidBodyRecordable,
        export_stage_snapshot,
    )

    from fasim.replicator.sdg.config_io import load_robot_config, load_task_config

    robot = load_robot_config(robot_key)
    task = load_task_config(
        robot_key, task_key,
        output_override=Path(output_dir) if output_dir else None,
    )

    out = Path(task.output_dir)
    out.mkdir(parents=True, exist_ok=True)

    try:
        snap = export_stage_snapshot(str(out))
        print(f"[recorder] stage snapshot: {snap}")
    except Exception as exc:
        print(f"[recorder] snapshot failed (non-fatal): {exc}")

    recorder = EpisodeRecorder(
        output_dir=str(out),
        session_metadata={
            "task_description": task.task_key,
            "robot_key": robot.robot_key,
            "source": "gui_live_ocs2",
            "capture_fps": task.capture_fps,
            "physics_dt": task.physics_dt,
        },
    )
    recorder.add(
        ArticulationRecordable(group=f"state/{robot.robot_key}", prim_path=robot.articulation_prim)
    )
    for label, prim in task.tracked_prims.items():
        recorder.add(RigidBodyRecordable(group=f"state/{label}", prim_path=prim))

    hdf5 = recorder.open_session()
    episode = recorder.start_episode()

    _STATE.update(
        recorder=recorder, hdf5=hdf5, episode=episode, out=str(out), frames=0
    )

    print(f"[recorder] session: {hdf5}")
    print(f"[recorder] episode {episode} ARMED (non-blocking)")
    print("[recorder] -> now run motion-generation in a terminal")
    print("[recorder] -> when the motion finishes, call stop() in this Script Editor")
    return {"hdf5": str(hdf5), "episode": episode, "output_dir": str(out)}


def stop(*, success: bool = True) -> dict:
    """Finish and close the armed session. Call this when the motion is done.

    Returns:
        ``{"hdf5":..., "episode":..., "output_dir":...}``.
    """
    rec = _STATE["recorder"]
    if rec is None:
        print("[recorder] nothing armed")
        return {}

    hdf5 = _STATE["hdf5"]
    episode = _STATE["episode"]
    out = _STATE["out"]
    try:
        rec.end_episode(success=success)
    finally:
        rec.close_session()
        _STATE.update(recorder=None, hdf5=None, episode=None, out=None, frames=0)

    print(f"[recorder] DONE episode={episode} hdf5={hdf5}")

    # report the real frame count so you can tell 0-frame sessions immediately
    try:
        import h5py

        with h5py.File(str(hdf5), "r") as f:
            grp = f[f"episodes/episode_{episode:05d}/state/{_first_group(f, episode)}"]
            for k in grp:
                print(f"[recorder]   {k}: {grp[k].shape}")
    except Exception as exc:
        print(f"[recorder] (could not report shapes: {exc})")

    print(f"\n[recorder] next: capture from a terminal (no ROS2 needed)")
    print(f"  cd {SDG_EXT}")
    print(f"  ./scripts/sdg.sh capture --robot unitree_g1d --task sinopec_test_tube --session {hdf5}")
    return {"hdf5": str(hdf5), "episode": episode, "output_dir": out}


def _first_group(f, episode: int) -> str:
    base = f"episodes/episode_{episode:05d}/state"
    return next(iter(f[base].keys()))


def status() -> None:
    """Print whether a session is currently armed."""
    if _STATE["recorder"] is None:
        print("[recorder] idle")
    else:
        print(f"[recorder] ARMED episode={_STATE['episode']} hdf5={_STATE['hdf5']}")


# ---------------------------------------------------------------------------
# convenience: timed recording that keeps Kit responsive
# ---------------------------------------------------------------------------
def arm_for(seconds: float, **kwargs) -> dict:
    """Arm, pump Kit for ``seconds``, then stop. Blocking but UI stays alive.

    Use inside the Script Editor when you prefer fire-and-forget::

        arm_for(60)

    Unlike a naive ``while time.sleep()`` this calls ``app.update()`` in the wait
    loop, so physics/rendering/OCS2 keep running.
    """
    import time

    import omni.kit.app

    info = arm(**kwargs)
    if not info:
        return {}

    app = omni.kit.app.get_app()
    print(f"[recorder] auto-stop in {seconds:.0f}s (Kit stays responsive)")
    deadline = time.monotonic() + seconds
    while time.monotonic() < deadline:
        app.update()          # keep the app loop alive — this is the key fix

    return stop()
