"""FaSim Replicator SDG — offline synthetic data capture for FaSim robots.

Two-phase workflow (decoupled so capture never needs ROS2):

    phase 1  record   articulation / rigid-body state -> HDF5
             trajectory source is whatever publishes /joint_states
             (OCS2 today, VR teleop later). Only primitive reads.

    phase 2  capture  replay the HDF5 headlessly and render annotators
             (RGB-D, camera params) -> dataset frames.

Importing this package must stay cheap: ``omni.*`` / ``carb`` are only pulled in
by :func:`record_episode` / :func:`capture_session`, so config-only commands
(``list`` / ``describe``) run under plain CPython.
"""

from __future__ import annotations

from typing import Any

from .annotators import ANNOTATOR_REGISTRY, describe_annotators, resolve_annotators
from .config_io import (
    SdgConfigError,
    load_default_config,
    load_robot_config,
    load_task_config,
)
from .types import CameraSpec, CaptureResult, RecordResult, RobotSpec, TaskSpec

__all__ = [
    "ANNOTATOR_REGISTRY",
    "CameraSpec",
    "CaptureResult",
    "RecordResult",
    "RobotSpec",
    "SdgConfigError",
    "TaskSpec",
    "capture_session",
    "describe_annotators",
    "load_default_config",
    "load_robot_config",
    "load_task_config",
    "record_episode",
    "resolve_annotators",
]


def __getattr__(name: str) -> Any:
    """Lazily expose Isaac-dependent callables (needs ``carb``/``omni``)."""
    if name == "record_episode":
        from .recorder import record_episode

        return record_episode
    if name == "capture_session":
        from .capturer import capture_session

        return capture_session
    raise AttributeError(f"module {__name__!r} has no attribute {name!r}")
