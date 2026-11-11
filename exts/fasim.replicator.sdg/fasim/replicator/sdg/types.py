"""Dataclasses shared by the recorder and capturer."""

from __future__ import annotations

from dataclasses import dataclass, field
from pathlib import Path
from typing import Any


@dataclass(frozen=True)
class CameraSpec:
    """One camera to render during phase 2."""

    name: str
    prim_path: str | None = None
    """Existing camera prim. ``None`` means create one from ``create``."""
    create: dict[str, Any] | None = None
    """For generated cameras: ``{"position": [...], "look_at": [...]}``."""
    resolution: tuple[int, int] = (640, 480)
    annotators: tuple[str, ...] = ("rgb",)
    depth: bool = True
    """Append ``distance_to_image_plane`` when True (idempotent)."""

    def effective_annotators(self) -> tuple[str, ...]:
        names = list(self.annotators)
        if self.depth and "distance_to_image_plane" not in names:
            names.append("distance_to_image_plane")
        return tuple(names)


@dataclass(frozen=True)
class RobotSpec:
    """Robot-level facts that do not change between tasks."""

    robot_key: str
    articulation_prim: str
    base_prim: str
    joint_state_topic: str = "/joint_states"
    ee_prims: dict[str, str] = field(default_factory=dict)
    #: 末端位姿的读取来源：``{side: {"link": 刚体路径, "offset": [x,y,z]}}``。
    #:
    #: 为什么不能直接读 ``ee_prims`` 里的 tcp：tcp 是纯 Xform（fixed joint 的
    #: 子 frame），而 newton kit 关了 PhysX→USD/Fabric 同步
    #: （``updateToUsd=false`` / ``fabricUpdateTransformations=false``），
    #: 所以 ``XformPrim.get_world_poses()`` 任何后端都读到 authoring 静态值。
    #:
    #: 解法：读 tcp 的**父刚体**（``RigidPrim`` 走 tensor 后端直读 PhysX，
    #: 不经过同步管道），再叠一个 URDF 里的固定偏移（tcp_joint 是 fixed）。
    ee_links: dict[str, dict[str, Any]] = field(default_factory=dict)
    gripper_joints: tuple[str, ...] = ()
    expected_dof_count: int | None = None


@dataclass(frozen=True)
class TaskSpec:
    """Task-level facts: scene, tracked objects, cameras, output."""

    task_key: str
    scene_usda: Path
    tracked_prims: dict[str, str] = field(default_factory=dict)
    cameras: tuple[CameraSpec, ...] = ()
    output_dir: Path = Path(".")
    capture_fps: int = 30
    physics_dt: float = 0.005
    max_frames: int = 2000
    variant_selection: dict[str, str] = field(default_factory=dict)
    """Variant switches applied to the robot prim before play, e.g. {"ROS": "fiveages"}."""


@dataclass
class RecordResult:
    hdf5_path: Path
    snapshot_path: Path | None
    frames: int
    duration_sec: float
    warnings: list[str] = field(default_factory=list)


@dataclass
class CaptureResult:
    output_dir: Path
    frames_written: int
    cameras: list[str]
    warnings: list[str] = field(default_factory=list)
