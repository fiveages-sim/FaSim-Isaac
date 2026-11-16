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
    """单 episode 帧数上限（防跑飞）。

    注意：这是**单次采集**的上限，不是数据集总量。批量导出会合并多个会话。
    """
    capture_presets: dict[str, dict] = field(default_factory=dict)
    """采集预设 —— 一组「保证数据格式对齐」的参数组合。

    为什么需要：同一个任务要采很多条轨迹（ACT 需要几十条）。如果每次采集都
    手选参数，很容易某次改了个分辨率或 every，那一批数据就和之前的不兼容了
    （``capture_profile`` 会在导出时报不一致，但那时已经浪费了采集时间）。

    预设把参数固化在任务配置里，选一次就锁定：

    ```yaml
    capture_presets:
      act_train:                     # 预设名
        desc: ACT 训练用（20fps，高分辨率）
        what: [camera, joint, ee, object]
        camera: all
        target_fps: 20               # 由 physics_hz 反推 every
        width: 1280
        height: 720
        depth: true
      act_fast:
        desc: 快速验证（10fps，低分辨率）
        what: [camera, joint]
        camera: all
        every: 20
        width: 640
        height: 480
    ```

    ``target_fps`` 与 ``every`` 二选一：前者按任务 ``physics_hz`` 自动换算，
    更稳（换场景步频变了也不会错）。都没写时用 ``every=10``。
    """
    variant_selection: dict[str, str] = field(default_factory=dict)
    """Variant switches applied to the robot prim before play, e.g. {"ROS": "fiveages"}."""

    randomize: dict = field(default_factory=dict)
    """运行时物体随机化配置（``randomize`` 段），用于自动化采集。

    **不与具体机器人/任务强绑定** —— 随机化哪些 prim、每个轴的范围、谁跟随谁，
    全部由这里声明。没有这一段就完全不做随机化（零影响）。

    ```yaml
    randomize:
      seed: 0                    # 0 = 每次用时间戳
      settle_steps: 60           # 设完位姿后让物理跑多少步（200Hz 下 60=0.3s）
      verify_tol_m: 0.05         # 读回验证容差（set_world_poses 可能被物理改写）
      objects:
        # 试管架：只在桌面内平移（z 由桌面决定，不要随机）
        - prim: /World/room/scene/test_tube_rack
          axes: {x: 0.04, y: 0.03}
        # 架上的试管：跟着架子一起搬（保持相对位置，避免穿模/甩飞）
        - prim: /World/room/scene/test_tube
          follow: /World/room/scene/test_tube_rack
    ```

    **顺序语义**：每轮总是先「恢复初始快照位姿」再叠加随机偏移。
    任务结尾物体位置和开头往往差很多（被抓到手里、插进孔里），
    直接对当前位置加偏移会把物体甩飞。

    实现在 :mod:`fasim.replicator.sdg.randomize`，
    可行性验证脚本 ``scripts/verify_randomize.py``。
    """


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
