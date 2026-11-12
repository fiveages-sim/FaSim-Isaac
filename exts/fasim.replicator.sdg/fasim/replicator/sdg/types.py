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
    #: 运动学配置（只用于把关节**指令**角换算成末端**指令**位姿）。
    #: ``{"urdf": "auto"|"none"|<path>, "cache_dir": str, "ee_frames": {side: name}}``
    #:
    #: 为什么需要：EE 回放必须发指令位姿，否则运控会二次叠加稳态误差
    #: （实测末端比指令低 7~9mm，表现为"抓取偏下"）。指令位姿只在运控 MPC
    #: 内部，Isaac 侧只有关节指令角，所以要用运控那份 planning URDF 做 FK。
    #:
    #: 迁移新机器人：改 ``ee_frames``（对应 .info 的 eeFrame/eeFrame1）即可，
    #: ``urdf: auto`` 会从运控缓存自动找。
    planning: dict[str, Any] = field(default_factory=dict)
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
    physics_hz: float | None = None
    """本任务期望的**物理步频**（Hz），来自场景 PhysicsScene 的
    ``physxScene:timeStepsPerSecond``。

    为什么需要显式写：物理步频定义在**场景资产**里，不同任务用不同场景就会用
    不同步频。采集器挂的是物理步回调，``every``（每 N 个物理步采一帧）的实际
    帧率 = ``physics_hz / every`` —— 不知道 physics_hz 就没法从"我要 30fps"
    反推 every，只能给用户几个猜出来的档位。

    采集开始时还会拿它与场景实测值对账（``SimulationManager.get_physics_dt()``），
    不一致会告警 —— 声明与资产不符时，时间轴会整体偏。
    """
    physics_hz_wall_baseline: float | None = None
    """本机**墙钟**物理步频实测基线（Hz），仅用于启动日志的落盘帧率预估。

    本机约 177（RTF≈0.89）。**绝不参与时间轴换算** —— 只回答"这台机器
    跑得动多少"。换机器要重新实测后改 ``configs/_default.yaml``。
    """
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
