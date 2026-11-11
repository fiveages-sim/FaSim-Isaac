"""层2：按 episode 的关节角复现机器人动作，并用 Replicator 渲染相机数据。

核心思路
========
编排（OCS2）本身就是**关节角控制**。所以只要：

  1. 同一套场景/资产（初始位姿一致）
  2. 逐帧把关节角驱动到录制值（**走 drive**，不是 teleport）
  3. **让物理真正步进**

物理就会自己把过程重演一遍 —— 夹爪位姿和录制时一致 ⇒ 接触力一致
⇒ 试管被自然抓起、交接、插入。**不需要手动摆物体位姿。**

注意第 2 条必须是 drive：`set_dof_positions`（teleport）每帧瞬移 5~20mm，
远大于接触厚度（2mm），接触约束每帧被打断，摩擦求解不出来，物体就会
「完全光滑地滑过」夹爪。详见 ``arm_gain`` 处的实测数据。

为什么不能手动摆物体（实测）
============================
   kinematic=True  + set_world_poses → 完全无效（设 +20cm，实际偏差 200.0mm）
   kinematic=False + set_world_poses → 被物理改写（位置飞到 (2.03,-1.88,0.93)）

``kinematic`` 刚体由物理引擎接管，API 写入被忽略；非 kinematic 则每帧被
重力和接触冲量改写。两条路都走不通，所以改为「不干预，让物理演化」。

职责边界
========
本模块只做：
  1. 逐帧驱动关节复现录制姿态：**走 drive**（``set_dof_position_targets`` +
     gain），与运控一致。用 ``set_dof_positions`` 瞬移会让接触约束每帧被打断，
     摩擦求解不出来，物体像「光滑滑过」夹爪 —— 详见 ``arm_gain`` 处的说明。
  2. ``app.update()`` 让物理步进（物体靠这一步自然响应）
  3. ``orchestrator.step(pause_timeline=False)`` 出图

它**不**启动控制器、**不**连 ROS2、**不**跑 OCS2、**不**手动写物体位姿。
底盘相关 DOF（驱动轮/脚轮）不驱动 —— 底盘姿态不在轨迹里，驱动它会拖走底座。

为什么用 DOF 而不是 link 位姿
=============================
Isaac 自带的 ``ArticulationRecordable`` 记的是 **47 个 link 的世界位姿**。
回放时用 ``set_world_poses`` 写回去对关节机器人是**物理非法的**：关节约束
被破坏，夹爪（``AssemblerFixedJoint`` 焊接）会被扯开。用 DOF 驱动则合法。

时间对齐
========
用**帧索引**对齐，不用时间戳。第 i 帧的图像就是第 i 帧关节角渲染出来的。
``pause_timeline`` 必须为 False —— 暂停会让物理停住，物体就不会被带起来。

从中间帧开始（``start > 0``）时会先静默推进 0..start 建立正确的物理状态，
否则物体初态是错的（试管可能悬空、夹爪可能已穿过它）。
"""

from __future__ import annotations

import time
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any, Callable

import numpy as np

from .episode import (
    GRIPPER_NAME_TOKENS,
    Episode,
    build_dof_mapping,
    skipped_dof_names,
)


def _log(msg: str) -> None:
    print(f"[SDG:render] {msg}", flush=True)


def _make_kinematic(prim_path: str) -> None:
    """把刚体设为 kinematic，并关掉 CCD（消除那条例行警告）。

    为什么需要：层2 是「姿态重放」，物体位置由录制数据决定，不能让物理参与。
    实测（未设 kinematic 时）：

        设定 (1.8797, 0.2341, 0.7430)
        读回 (1.8697, 0.2028, 0.7722)   ← y 偏 3cm、z 偏 3cm

    偏移不是单纯下落，而是**机器人夹爪把它推开了**（碰撞生效）。

    kinematicEnabled=True 后，刚体不受重力/碰撞冲量影响，
    ``set_world_poses`` 设定的位姿会保持。

    注意：PhysX 会对 kinematic + CCD 报错「kinematic bodies with CCD enabled
    are not supported」，所以这里一并关掉 CCD。
    """
    from pxr import UsdPhysics

    import omni.usd

    stage = omni.usd.get_context().get_stage()
    prim = stage.GetPrimAtPath(prim_path)
    if not prim or not prim.IsValid():
        raise RuntimeError(f"prim not found: {prim_path}")

    rb = UsdPhysics.RigidBodyAPI(prim)
    if not rb:
        rb = UsdPhysics.RigidBodyAPI.Apply(prim)

    rb_attr = rb.GetKinematicEnabledAttr()
    if not rb_attr:
        rb_attr = rb.CreateKinematicEnabledAttr()
    rb_attr.Set(True)

    # Kinematic 与 CCD 互斥，关掉以免 PhysX 每次报错
    for attr_name in (
        "physxRigidBody:enableCCD",
        "physxRigidBody:gravityScale",
    ):
        if prim.HasAttribute(attr_name):
            try:
                is_gravity = "gravity" in attr_name
                prim.GetAttribute(attr_name).Set(0.0 if is_gravity else False)
            except Exception:  # noqa: BLE001
                pass


def _episode_frame_dt(episode: Any) -> float:
    """取 episode 的典型帧间隔（秒）。用于把回放速度对齐到录制速度。"""
    try:
        t = np.asarray(episode.t, dtype=np.float64)
        if t.size >= 2:
            d = np.diff(t)
            d = d[np.isfinite(d) & (d > 0)]
            if d.size:
                return float(np.median(d))
    except Exception:  # noqa: BLE001
        pass
    return 0.02


def _lock_base(robot_config: Any) -> None:
    """把机器人底盘钉在世界坐标系，复现层1 录制时的 BASE_LOCK。

    为什么必须做
    ============
    层1 编排全程 **BASE_LOCK**：OCS2 主动控住底盘，手臂反作用力被抵消，
    你手动跑时底盘纹丝不动。而回放的 USD 里 ``base_link`` 是**自由浮动
    刚体**，没有任何东西约束它 —— 手臂一挥手，整台机器被反作用力矩带着
    慢速偏航/漂移。

    实测（pick_10 回放，只驱动关节角）：
      - 关节跟踪误差 0.00 mrad（teleport 精确）
      - 但 base_link 到 t=28s 已偏航 **-12.5°**、位移 11 mm
      - 右末端因此偏离录制位姿 **76 mm**，夹爪从试管旁擦过去

    视觉表现就是：机器人「想」伸向试管，但整个人在慢慢转，手最后落在
    试管旁边的空中。

    实现要点
    ========
    用 ``PhysicsFixedJoint`` 把 ``base_prim`` 锚到世界（``body0`` 留空）。
    **必须在构造 ``Articulation`` 之前调用** —— fixed joint 会改变
    articulation 拓扑，之后再加会让 physics tensor 失效（关节读写全变
    NaN、画面定格）。
    """
    import omni.usd
    from pxr import Gf, UsdPhysics

    base_prim = str(getattr(robot_config, "base_prim", "") or "")
    if not base_prim:
        _log("WARN robot_config 没有 base_prim，跳过锁底盘")
        return
    stage = omni.usd.get_context().get_stage()
    if stage is None or not stage.GetPrimAtPath(base_prim):
        _log(f"WARN 找不到 base_prim {base_prim}，跳过锁底盘")
        return
    joint_path = base_prim.rsplit("/", 1)[0] + "/_ReplayFixedBase"
    if stage.GetPrimAtPath(joint_path):
        return
    joint = UsdPhysics.FixedJoint.Define(stage, joint_path)
    joint.CreateBody1Rel().SetTargets([base_prim])
    # body0 不设 = 世界坐标系；local 变换全零 = 保持当前世界位姿
    for attr, value in (
        ("LocalPos0Attr", Gf.Vec3f(0, 0, 0)),
        ("LocalPos1Attr", Gf.Vec3f(0, 0, 0)),
        ("LocalRot0Attr", Gf.Quatf(1, 0, 0, 0)),
        ("LocalRot1Attr", Gf.Quatf(1, 0, 0, 0)),
    ):
        try:
            getattr(joint, "Create" + attr)().Set(value)
        except Exception:  # noqa: BLE001
            pass
    _log(f"锁底盘: {base_prim} → PhysicsFixedJoint@{joint_path}")


def _build_gripper_switches(
    dof_names: list[str],
    target_idx: np.ndarray,
    source_idx: np.ndarray,
    joint_names: list[str],
    joint_pos: np.ndarray,
    art: Any,
) -> list[dict[str, Any]]:
    """把夹爪识别成「开/合 + 夹持力」，返回驱动规格。

    背景（对齐运控 ``AdaptiveGripperController`` 的实现）
    =====================================================
    运控用 ``position_command`` 驱动夹爪，最终经
    ``IsaacArticulationController`` 落到 ``set_dof_position_targets``
    —— 即 **走 drive 产生力**，不是 teleport。层2 必须用同一个 API。

    夹爪行程的真实形状（Dex1 实测，``ROS=fiveages`` 变体下与编排同环境）
    ---------------------------------------------------------------
    两指**对向**移动，间距只取决于 ``|j|``::

        j = +0.0245 (上限)  → 两指间距 49.0 mm   ← 最开
        j = +0.0100          → 20.0 mm
        j =  0.0000          →  0.0 mm           ← 完全闭合
        j = -0.01495         → 29.9 mm           ← 录制里的"闭合值"
        j = -0.0200 (下限)   → 40.0 mm           ← 反向张开

    即 ``gap ≈ 2000 * |j|`` mm：**0 是闭合点，正负两侧都是张开**。
    URDF 里两指轴相反（``xyz="0 -1 0"`` / ``0 1 0``）+ mimic
    ``multiplier=1``，正是这个语义。

    所以「压到限位 = 夹紧」是错的（压到 -0.02 反而张开到 40mm）。录制里
    闭合值 -0.01495 对应 29.9mm，正是被 16mm 试管撑开后的平衡位置 ——
    这才是真实夹持状态。

    回放策略
    ========
    - 开：目标 = 录制的张开值（实测 +0.0245，即上限）
    - 合：目标 = 录制的闭合值**原样**（实测 -0.0149，对应两指间距 29.9mm，
      正是被 16mm 试管撑开的平衡位置）。运控侧 ``AdaptiveGripperController``
      带力反馈（``force_threshold`` + ``force_feedback_ratio``），碰到物体
      就把目标回退一半 —— 录制里的闭合值本身已经是带夹持力的平衡点，
      原样驱动即可复现，不需要额外加压。

    开关判定用录制角度是否远离张开值，避免采集中途 NaN 抖动。
    """
    try:
        limits = art.get_dof_limits()
        lo_all = np.asarray(limits[0], dtype=np.float64).reshape(-1)
        hi_all = np.asarray(limits[1], dtype=np.float64).reshape(-1)
    except Exception as exc:  # noqa: BLE001
        _log(f"WARN 读不到关节限位，夹爪退化为位置回放: {exc}")
        return []

    specs: list[dict[str, Any]] = []
    for k in range(len(source_idx)):
        name = str(joint_names[source_idx[k]])
        if not any(tok in name.lower() for tok in GRIPPER_NAME_TOKENS):
            continue
        dof = int(target_idx[k])
        col = int(source_idx[k])
        # mimic 关节限位是 [0, 0]，跳过（由 mimic API 自行跟随主关节）
        if hi_all[dof] - lo_all[dof] < 1e-6:
            continue

        lo, hi = float(lo_all[dof]), float(hi_all[dof])
        col_vals = np.asarray(joint_pos[:, col], dtype=np.float64)
        valid = np.isfinite(col_vals)
        if not valid.any():
            continue

        vals = col_vals[valid]
        open_val = float(np.percentile(vals, 98))
        closed_val = float(np.percentile(vals, 2))
        span = abs(hi - lo)
        # 「显著闭合」的门槛：低于它才算真的夹住东西
        close_thr = open_val - 0.60 * span

        has_close = closed_val < close_thr

        closed = np.zeros(col_vals.shape, dtype=bool)
        if has_close:
            closed[valid] = col_vals[valid] < close_thr
            idx_valid = np.where(valid, np.arange(len(col_vals)), 0)
            np.maximum.accumulate(idx_valid, out=idx_valid)
            closed = closed[idx_valid]
        else:
            _log(f"夹爪 {name}: 录制里没有明显闭合段（2% 分位 {closed_val:+.4f} "
                 f">= 阈值 {close_thr:+.4f}），全部按位置回放")

        # 驱动目标：
        #   闭合帧 → 关节下限（压出夹持力，与运控一致）
        #   其余   → **录制里该帧的原值**（保留 0.0 中立态、保留张开的中间过程）
        close_target = float(np.clip(lo, lo, hi))

        n_closed = int(closed.sum())
        _log(
            f"夹爪 {name}: 录制值域 [{closed_val:+.4f}, {open_val:+.4f}] "
            f"限位 [{lo:+.4f}, {hi:+.4f}]"
        )
        if has_close:
            _log(f"  闭合 {n_closed}/{len(closed)} 帧 → 目标 {close_target:+.4f}；"
                 f"其余帧按录制原值驱动")
        specs.append(
            {
                "name": name,
                "dof": dof,
                "col": col,
                "lo": lo,
                "hi": hi,
                "open_target": open_val,
                "close_target": close_target,
                "closed": closed,
                # 录制原值：非闭合帧直接用它（保留 0.0 中立态与张开的中间过程）
                "raw_values": col_vals.astype(np.float32),
            }
        )
    return specs


def _gripper_target(spec: dict, frame: int) -> float:
    """夹爪该帧的驱动目标。

    - 闭合帧 → 关节下限（压出夹持力，与运控一致）
    - 其余帧 → **录制里该帧的原值**

    为什么非闭合帧要用原值，而不是统一"张开"：
    录制开头夹爪常停在 0.0（运控未下指令时的自然位置），之后才张到
    +0.0245。若把 0.0 也当成"张开"驱动到 +0.0245，回放一开始夹爪就
    全开了 —— 与编排现象不符（用户实测）。

    原值里可能有 NaN（采集缺失），那时往前找最近的有效值。
    """
    closed = spec.get("closed")
    idx = min(max(0, frame), len(closed) - 1) if closed is not None else 0
    if closed is not None and bool(closed[idx]):
        return float(spec["close_target"])
    raw = spec.get("raw_values")
    if raw is not None and 0 <= idx < len(raw):
        v = float(raw[idx])
        if np.isfinite(v):
            return v
        for k in range(idx - 1, -1, -1):
            vv = float(raw[k])
            if np.isfinite(vv):
                return vv
    return float(spec["open_target"])


def _measure_app_dt(app: Any, samples: int = 10) -> float:
    """实测「一次 app.update() 推进多少仿真秒」。

    默认是 16.667 ms（kit player 按 60 Hz 渲染时间推进）。
    必须实测，不能靠 1/tcps 推算；也不要改成 fixed time stepping：
    实测那样时间轴会走但 PhysX 不 step，回放完全定格。
    """
    import omni.timeline

    timeline = omni.timeline.get_timeline_interface()
    was_playing = timeline.is_playing()
    try:
        if not was_playing:
            timeline.play()
        for _ in range(5):  # 预热，跳过首帧开销
            app.update()
        t0 = float(timeline.get_current_time())
        for _ in range(samples):
            app.update()
        t1 = float(timeline.get_current_time())
        dt = (t1 - t0) / samples
        return dt if dt > 0 else 0.0
    finally:
        if not was_playing:
            timeline.pause()


@dataclass
class RenderResult:
    """一次渲染采集的结果。"""

    episode_path: str
    output_dir: str
    frames_written: int
    cameras: list[str]
    dof_names: list[str]
    skipped_dofs: list[str]
    missing_joints: list[str]
    joint_max_error: float
    wall_seconds: float
    extras: dict[str, Any] = field(default_factory=dict)


def render_episode(
    *,
    episode_path: str | Path,
    robot_config: Any,
    task_config: Any,
    output_dir: str | Path | None = None,
    app: Any = None,
    start: int = 0,
    end: int | None = None,
    steps_per_frame: int = 0,
    verify_joints: bool = True,
    on_frame: Callable[[int, dict], None] | None = None,
    render_indices: set[int] | None = None,
    lock_base: bool = True,
    arm_gain: tuple[float, float] | None = (100000.0, 10000.0),
    gripper_drive: tuple[float, float, float] | None = (50000.0, 2000.0, 25.0),
) -> RenderResult:
    """回放 episode 并采集相机数据。

    Args:
        episode_path: 层1 产出的 npz。
        robot_config: ``RobotSpec``（articulation prim、相机定义）。
        task_config: ``TaskSpec``（相机列表、输出目录）。
        output_dir: 覆盖 ``task_config.output_dir``。
        app: 运行中的 ``SimulationApp``；``None`` 时用
            ``isaacsim.simulation_app`` 启动一个无头实例。
        start / end: 帧区间（调试用）。**注意**：物体由物理驱动，
            从中间帧开始会得到错误的物理初态。需要看某一段时，程序会
            自动从 0 静默推进到 ``start``（只算物理，不出图），
            所以语义是「从开始播放到 start 再开始采集」。
        steps_per_frame: 每帧 ``app.update()`` 次数。**默认 0 = 自动对齐**：
            按 episode 的真实帧间隔推进仿真时间，使回放速度与录制一致。
            物理接触对速度极其敏感，速度错 2 倍就抓不住物体。只有需要
            刻意加速/减速排查时才手动指定。
        verify_joints: 每帧回读 ``get_dof_positions`` 并与目标比对，
            报告最大误差。这是判断"回放是否忠实"的核心指标。
        on_frame: 每帧回调 ``(idx, info)``，用于外部记录。
    """
    import carb
    import omni.replicator.core as rep
    import omni.usd

    episode = Episode.load(episode_path)
    episode.validate()
    out = Path(output_dir or task_config.output_dir)
    out.mkdir(parents=True, exist_ok=True)

    n_total = len(episode)
    n_start = max(0, int(start))
    n_end = n_total if end is None else min(n_total, int(end))
    if n_end <= n_start:
        raise ValueError(f"空区间: start={n_start} end={n_end} (总帧数 {n_total})")
    if n_start > 0:
        _log(
            f"注意: start={n_start} > 0，但物体靠物理演化。"
            f"将先静默推进 0..{n_start} 建立正确的物理状态（不出图）。"
        )

    _log(f"episode: {episode_path}")
    _log(f"帧区间: [{n_start}, {n_end}) / {n_total}")
    for line in episode.summary().splitlines():
        _log(f"  {line}")

    # ------------------------------------------------------------------ 场景
    owns_app = False
    if app is None:
        from isaacsim.simulation_app import SimulationApp

        app = SimulationApp({"headless": True})
        owns_app = True
        _log("已启动无头 SimulationApp")

    # 打开场景并切 variant。**场景与层1 录制时保持一致**（含 ROS 变体，
    # 即 ros2_control 也在环内），这样回放和你手动跑编排是同一个环境，
    # 差异只来自「关节角怎么被驱动」。
    from .recorder import _apply_variants, _open_stage

    _open_stage(Path(task_config.scene_usda))
    variants = dict(getattr(task_config, "variant_selection", {}) or {})
    # 场景与层1 录制时保持一致（含 ROS 变体），让差异只来自「关节角怎么驱动」。
    if variants:
        _apply_variants(robot_config.articulation_prim, variants)

    # 锁底盘：复现层1 的 BASE_LOCK。不锁的话手臂反作用力会让整台机器
    # 慢速偏航（实测 12.5°），末端偏 76mm，手就落在试管旁边的空中。
    # **必须在 Articulation 构造之前**，否则 physics tensor 失效。
    if lock_base:
        _lock_base(robot_config)
    for _ in range(3):
        app.update()

    # ------------------------------------------------------------------ 关节映射
    from isaacsim.core.experimental.prims import Articulation

    art = Articulation(robot_config.articulation_prim)
    dof_names = list(art.dof_names)
    _log(f"articulation: {robot_config.articulation_prim}  DOF={len(dof_names)}")

    target_idx, source_idx, missing = build_dof_mapping(dof_names, episode.joint_names)

    # 剔除整列 NaN 的关节（典型是 URDF 的 mimic 关节，如 Dex1 的
    # *_gripper_joint1：ros2_control 不为它声明 state interface，
    # /joint_states 里根本没有这个 name，采集端只能填 NaN）。
    #
    # 这类关节在 USD 上挂了 PhysxMimicJointAPI / NewtonMimicAPI，
    # 物理会自己让它们跟随主关节，**不能**手动写入，否则与 mimic 打架。
    if len(source_idx):
        col_finite = np.isfinite(episode.joint_pos[:, source_idx]).any(axis=0)
        dead = [episode.joint_names[c] for c, ok in zip(source_idx, col_finite) if not ok]
        if dead:
            _log(f"跳过整列为 NaN 的 mimic 关节 {len(dead)} 个（物理自行跟随）: {dead}")
            target_idx = target_idx[col_finite]
            source_idx = source_idx[col_finite]
    if len(source_idx) == 0:
        raise RuntimeError(
            "episode 里没有任何一列有有效关节角 —— 数据无效，拒绝渲染。"
        )
    skipped = skipped_dof_names(dof_names)
    # 底盘（轮子/脚轮）**必须从驱动列表里剔除**，否则：
    #   - 轮子被当位置目标驱动，会和物理/底盘控制打架
    #   - 脚轮是被动关节，强行驱动会产生巨大跟踪误差（实测 74 mrad）
    if len(target_idx):
        non_manip = set(skipped_dof_names(dof_names))
        keep = [
            i for i, d in enumerate(target_idx)
            if dof_names[int(d)] not in non_manip
        ]
        dropped = len(target_idx) - len(keep)
        if dropped:
            _log(f"剔除底盘关节 {dropped} 个（不驱动）: "
                 f"{[dof_names[int(target_idx[i])] for i in range(len(target_idx)) if i not in keep]}")
            target_idx = target_idx[keep]
            source_idx = source_idx[keep]
    if missing:
        _log(f"WARN episode 里有 {len(missing)} 个关节在此 articulation 中找不到: {missing}")
        _log("     这些列会被忽略；回放后的姿态可能与录制不一致。")
    _log(f"映射: {len(target_idx)} 个关节将被驱动，跳过底盘 {len(skipped)} 个: {skipped}")
    if len(target_idx) == 0:
        raise RuntimeError(
            "没有任何关节名能匹配上 —— episode 与 articulation 不兼容，拒绝生成错误数据。"
        )

    # ------------------------------------------------------------ 夹爪：开/合 + 夹持力
    #
    # **为什么不能直接 teleport 回放夹爪关节角**（层2 最容易踩的坑）：
    #
    # 运控（``AdaptiveGripperController`` → ``IsaacArticulationController``）
    # 用 ``set_dof_position_targets`` 驱动夹爪，即 **走 drive 产生力**。
    # 闭合时 target 继续往里压，手指碰到物体后停住，于是
    # F = K*(target-actual) 产生夹持力 —— 这才是能把物体提起来的原因。
    # 而 /joint_states 记录的是被接触限制后的 **actual**。
    #
    # 若回放也用 actual 作为 target，target==actual ⇒ 夹持力恒为 0，物理上
    # 不可能抓住任何东西。所以必须还原「夹住」这个语义（见
    # ``_build_gripper_switches`` 里对 Dex1 行程形状的说明）。
    #
    # 做法：
    #   张开 → target = 录制的张开值（+0.0245）
    #   闭合 → target = 录制的闭合值再往合拢方向压 6mm ⇒ 持续施压产生夹持力
    # 必须用 set_dof_position_targets（走 drive），不能用 set_dof_positions
    #（teleport 不产生力，手指会直接穿过物体）。
    grip_specs = _build_gripper_switches(
        dof_names,
        target_idx,
        source_idx,
        list(episode.joint_names),
        episode.joint_pos,
        art,
    )
    # 夹爪不再按位置回放，从位置映射里剔除，避免 teleport 覆盖 drive 的结果。
    grip_dofs = {s["dof"] for s in grip_specs}
    if grip_dofs:
        keep = [k for k, d in enumerate(target_idx) if int(d) not in grip_dofs]
        target_idx = target_idx[keep]
        source_idx = source_idx[keep]
        _log(
            f"夹爪改开关量后，位置驱动关节数 = {len(target_idx)}"
            f"（剔除 {len(grip_dofs)} 个夹爪 DOF）"
        )

    # ------------------------------------------------------------------ 时间基准
    #
    # 物理接触对**回放速度**极其敏感：夹爪必须以录制时的速度接近试管，
    # 接触力才来得及建立。速度错了，同样是 3mrad 的关节角，夹爪会以
    # 高速"扫"过试管，物体会被推开而不是被夹住。
    #
    # 默认情况下 app.update() 固定推进 16.667 ms（60Hz 渲染时间），
    # 而录制通常是 50Hz（20 ms/帧）。若直接 steps_per_frame=3，
    # 每帧推进 ~50 ms —— 2.5 倍速回放，物理抓不住。
    #
    # 对齐方法：打开 fixed time stepping，把 stage 的 timeCodesPerSecond
    # 设成物理步率的整数倍，使每次 app.update() 精确推进 1..N 个物理步。
    # 实测：
    #   tcps=60 → 16.667 ms/update
    #   tcps=50 + fixed → 10.000 ms/update
    #   tcps=200 + fixed →  5.000 ms/update
    # 之后由时间累积器决定每帧要 update 几次，使仿真时间与 episode 的
    # t 轴一致，而不是靠固定的 steps_per_frame 猜。
    frame_dt = _episode_frame_dt(episode)
    # 每次 app.update() 推进多少仿真秒由 kit 的 player 决定，默认 16.667 ms
    # （60 Hz）。**不能**改成 fixed time stepping 来对齐：实测那样时间轴
    # 会走但 PhysX 不再被 step（关节读回恒等于设定值、画面定格）。
    #
    # 正确做法是保留默认步进，用累积器把每帧的 update 次数凑成
    # 「总仿真时间 = 录制帧间隔」，这样平均速度与录制一致。
    app_dt = _measure_app_dt(app)
    if app_dt <= 0:
        app_dt = 1.0 / 60.0
    if steps_per_frame > 0:
        # 显式指定了就尊重，但仍然记录真实速度比。
        per_frame_updates = max(1, int(steps_per_frame))
        _log(
            f"时间基准: 手动 steps_per_frame={per_frame_updates}，"
            f"每次 update {app_dt*1000:.2f} ms → 每帧 {per_frame_updates*app_dt*1000:.1f} ms，"
            f"录制帧间隔 {frame_dt*1000:.1f} ms"
            f"（速度比 {per_frame_updates*app_dt/frame_dt:.2f}x）"
        )
    else:
        per_frame_updates = 0  # 交给累积器
        _log(
            f"时间基准: 自动对齐 录制帧间隔 {frame_dt*1000:.1f} ms，"
            f"每次 update {app_dt*1000:.2f} ms → 平均每帧 {frame_dt/app_dt:.2f} 次"
        )

    # ------------------------------------------------------------------ 锁底盘
    #
    # 层1 的编排是 **BASE_LOCK** 的：OCS2 让底盘保持在原地，手臂反作用力
    # 由底盘"吸收"（速度锁）。而回放的 USD 里 base_link 是**自由浮动刚体**，
    # 手臂一挥就会把整机带着转。
    #
    # 实测（pick_10 回放，只跑关节角）：
    #   关节跟踪误差 0.00 mrad（teleport 精确），但
    #   base_link 到 t=28s 已偏航 **-12.5°**，位移 11 mm，
    #   右末端因此偏离录制位姿 **76 mm** —— 夹爪从试管旁边擦过去，
    #   所以物理"抓不住"，物体偏差 250 mm。
    #
    # 结论：必须把底盘锚住，否则层2 的物理前提与层1 不同。
    # 用 PhysicsFixedJoint 把 base_link 钉在世界原点（body0 留空 = world）。
    #
    # ------------------------------------------------------------------ 相机
    cameras = _setup_cameras(rep, task_config)
    if not cameras:
        raise RuntimeError("没有可用的相机，无法采集")

    # ------------------------------------------------------------------ 手臂 drive
    #
    # **这是层2 能不能抓住物体的决定性因素。**
    #
    # 之前用 ``set_dof_positions``（teleport）驱动手臂：每帧瞬间把关节挪到
    # 录制位置，位移量 5~20mm。而接触厚度（contactOffset）只有 2mm —— 手指
    # 每一帧都在「穿透」物体，PhysX 来不及建立稳定的接触约束就被下一次瞬移
    # 打断，摩擦约束根本求解不出来。
    #
    # 表现就是：手指明明夹住了试管（`grip` 停在被挡住的位置），但一抬手试管
    # 就「完全光滑地滑过」夹爪 —— 因为摩擦在物理上等于不存在。
    #
    # 运控用的是 ``set_dof_position_targets``（drive），手臂连续运动、每帧位移
    # 远小于接触厚度，接触约束稳定存在，摩擦正常生效。
    #
    # 实测对比（pick_10，抓取帧 1050 → 抬手 1350）：
    #   teleport 手臂: grip -0.0146→-0.0200（空合），tube_z 0.7431→0.7430  ❌
    #   drive 手臂:    grip -0.0143→-0.0143（一直夹着），tube_z 0.7431→0.9119 ✅
    #                  （试管被抬起 169mm，tcp 上升 171mm）
    #
    # 所以：手臂也必须走 drive，与运控一致。
    if arm_gain:
        try:
            n = len(target_idx)
            k, d = float(arm_gain[0]), float(arm_gain[1])
            art.set_dof_gains(
                stiffnesses=np.full((1, n), k, dtype=np.float32),
                dampings=np.full((1, n), d, dtype=np.float32),
                dof_indices=target_idx,
            )
            _log(f"手臂 drive gain 已设置: K={k:g} D={d:g}（{n} 个关节）")
        except Exception as exc:  # noqa: BLE001
            _log(f"WARN 设置手臂 gain 失败，退回 teleport: {exc}")
            arm_gain = None

    # ------------------------------------------------------------------ 夹爪 drive
    #
    # **夹爪力气必须够，否则竖直提不起来。** 资产默认是
    # ``stiffness=10000 / damping=1000 / maxForce=20N``，实测这个配置
    # 只能把试管抬起 80mm 就滑落（抓取段偏差 923mm）。
    #
    # 实测扫参（pick_10，完整 2200 帧）：
    #   maxF=20N  → grip 停在 -0.0021（顶不住），试管掉到地上        ❌
    #   maxF=25N  → grip 稳定 -0.0147，抬起 186mm，p95=13.2mm        ✅
    #   maxF=40N  → 抬起 187mm，p95=9.1mm                            ✅
    #   maxF=80N  → 抬起 184mm，p95=16.6mm                           ✅（过大易抖）
    #
    # 临界点在 20~25N 之间。取 **25N** —— 只比 Dex1 的额定 effort
    # （20N）高一点，尽量贴近现实，也不至于因为过硬而抖。
    # stiffness 提到 50000 是必要的：它决定手指跟随手的速度，太软就追不上。
    if gripper_drive:
        try:
            k, d, mf = (float(x) for x in gripper_drive)
            from pxr import Usd

            import omni.usd as _ou

            stage = _ou.get_context().get_stage()
            n_set = 0
            for prim in Usd.PrimRange(stage.GetPrimAtPath(robot_config.articulation_prim)):
                if not prim.GetName().lower().endswith("gripper_joint"):
                    continue
                if not prim.GetAttribute("drive:linear:physics:type").IsValid():
                    continue
                for attr, val in (
                    ("drive:linear:physics:stiffness", k),
                    ("drive:linear:physics:damping", d),
                    ("drive:linear:physics:maxForce", mf),
                ):
                    a = prim.GetAttribute(attr)
                    if a and a.IsValid():
                        a.Set(float(val))
                n_set += 1
            if n_set:
                _log(f"夹爪 drive: K={k:g} D={d:g} maxForce={mf:g}N（{n_set} 个关节）")
        except Exception as exc:  # noqa: BLE001
            _log(f"WARN 设置夹爪 drive 失败，用资产默认值: {exc}")

    # ------------------------------------------------------------------ 物体
    #
    # 方案：**让物理自己算物体位姿，不手动摆位。**
    #
    # 为什么不 set_world_poses（两种都试过，都不可用）：
    #   kinematic=True  + set_world_poses → 完全无效（实测设 +20cm，实际偏差 200mm）
    #   kinematic=False + set_world_poses → 被物理改写（位置飞到 (2.03,-1.88,0.93)）
    #
    # 正确做法：timeline 播放，物理真正运行。录制时夹爪能抓住试管，
    # 回放时关节角一致 → 夹爪位姿一致 → 物理接触一致 → 试管自然跟随。
    # 前提是碰撞体正确（已确认 GlassBody/BlueCap 都有 CollisionAPI）。
    obj_views: dict[str, Any] = {}
    if episode.objects:
        from isaacsim.core.experimental.prims import RigidPrim

        prim_map = dict(getattr(task_config, "tracked_prims", {}) or {})
        for label in episode.objects:
            prim_path = prim_map.get(label)
            if not prim_path:
                _log(f"WARN 物体 {label!r} 在 task_config.tracked_prims 里没有对应 prim")
                continue
            try:
                obj_views[label] = RigidPrim(prim_path)
                _log(f"物体 {label}: {prim_path}（物理驱动，不手动设位姿）")
            except Exception as exc:  # noqa: BLE001
                _log(f"WARN 无法创建 RigidPrim({prim_path}): {exc}")

    # ------------------------------------------------------------------ 逐帧
    #
    # timeline 必须**播放**：set_dof_positions 依赖 physics tensor entity，
    # 而它只在 timeline 播放时才有效（暂停会报
    # "physics tensor entity is not valid. Play the simulation/timeline"）。
    #
    # timeline 必须**播放**，原因有两条：
    #   1. set_dof_positions 依赖 physics tensor entity，暂停时无效
    #   2. **物体靠物理自身响应**：夹爪关节角一致 → 接触力一致 → 试管被自然抓起
    #
    # 这是本方案的核心：不手动摆物体，让物理复现录制时的抓取。
    # 前提是同一场景 + 相同初始位姿 + 相同关节角轨迹 ⇒ 相同的物理演化。
    import omni.timeline

    timeline = omni.timeline.get_timeline_interface()
    if not timeline.is_playing():
        timeline.play()
        _log("timeline: play（关节 teleport + 物体由物理自然响应）")
    for _ in range(10):
        app.update()

    t_wall = time.monotonic()
    max_err = 0.0
    frames_written = 0
    errors_per_frame: list[float] = []
    obj_dev: dict[str, list[float]] = {}

    # 仿真时间累积器：每帧推进的量等于 episode 的帧间隔，而不是
    # 固定的 update 次数。这样 app.update() 的步长（由 tcps 决定）
    # 与录制的 50Hz 就严格一致，物理接触速度才和录制时相同。
    sim_clock = {"acc": 0.0}

    def _advance(dt_target: float, fixed_updates: int = 0) -> int:
        """推进仿真 ``dt_target`` 秒；``fixed_updates>0`` 时用固定次数。"""
        if fixed_updates > 0:
            for _ in range(fixed_updates):
                app.update()
            return fixed_updates
        sim_clock["acc"] += dt_target
        n = int(round(sim_clock["acc"] / app_dt))
        n = max(1, min(n, 50))  # 单帧上限 50 次，防止时间跳变时卡死
        for _ in range(n):
            app.update()
        sim_clock["acc"] -= n * app_dt
        # 累积器不追平负余额，避免长期误差累积成停顿
        sim_clock["acc"] = max(-app_dt, sim_clock["acc"])
        return n

    def _apply_joints(values: np.ndarray) -> None:
        """驱动手臂关节：有 gain 走 drive，否则退回 teleport。"""
        if arm_gain:
            art.set_dof_position_targets(values, dof_indices=target_idx)
        else:
            art.set_dof_positions(values, dof_indices=target_idx)

    def _apply_grippers(frame_i: int) -> None:
        """按开/合驱动夹爪（走 drive 产生夹持力，与运控同一协议）。"""
        for spec in grip_specs:
            state = bool(spec["closed"][min(frame_i, len(spec["closed"]) - 1)])
            target = _gripper_target(spec, frame_i)
            try:
                art.set_dof_position_targets(
                    np.array([[target]], dtype=np.float32),
                    dof_indices=np.array([spec["dof"]], dtype=np.int64),
                )
            except Exception as exc:  # noqa: BLE001
                _log(f"WARN 夹爪 {spec['name']} 设目标失败: {exc}")

    # ---- 静默推进：从 0 到 n_start 只算物理，不出图 ----
    #
    # 物体靠物理演化，从中间帧直接开始会得到错误的初态
    # （试管可能还悬空、夹爪可能已穿过它）。
    # 因此 start>0 时先把前面所有帧的关节角走一遍，让物理到达同一状态。
    if n_start > 0:
        _log(f"静默推进 0..{n_start}（只算物理，不出图）...")
        for j in range(0, n_start):
            row_j = episode.joint_pos[j][source_idx]
            if np.all(np.isfinite(row_j)):
                _apply_joints(row_j.astype(np.float32).reshape(1, -1))
            _apply_grippers(j)
            _advance(frame_dt, per_frame_updates)
            if (j + 1) % 200 == 0:
                _log(f"  推进 {j + 1}/{n_start}")
        _log("静默推进完成")

    _log(
        f"开始采集 {n_end - n_start} 帧 "
        f"(更新方式={'手动 ' + str(per_frame_updates) + ' 次/帧' if per_frame_updates else '自动对齐'}"
        f", verify_joints={verify_joints})"
    )

    try:
        for i in range(n_start, n_end):
            if i == n_start:
                _log(f"首帧 {i}: 写关节目标 ({len(target_idx)} 个)")
            # 1) 直接 teleport 关节到录制位置。
            #
            # 1) 驱动关节到录制位置（走 drive，见上方 arm_gain 说明）。
            #
            # 探针实测：
            #   set_dof_positions（teleport）→ 关节误差 <1mrad，但接触约束
            #       每帧被打断，摩擦失效，物体「光滑滑过」夹爪。
            #   set_dof_position_targets + gain → 关节误差稍大，但接触稳定，
            #       物体能被正常夹起（实测试管抬起 169mm）。
            # 抓取任务必须用后者，与运控保持一致。
            #
            # API 注意：indices = **prim** 索引，dof_indices = **关节** 索引。
            #
            # NaN 处理：采集开头运控未就绪的帧关节角是 NaN，跳过不写
            #（保持上一帧姿态），而不是把 NaN 灌进物理。
            row = episode.joint_pos[i][source_idx]
            values = None
            if np.all(np.isfinite(row)):
                values = row.astype(np.float32).reshape(1, -1)
                _apply_joints(values)
            # 夹爪：开关量驱动（合→压下限产生夹持力，开→到上限）
            _apply_grippers(i)

            # 2) 推进物理：关节 teleport 到位后，让物理把机器人链解算一遍，
            #    同时让物体（试管等）按接触力自然响应。
            #
            #    推进量按 episode 的真实帧间隔算（见 _advance），
            #    保证回放速度与录制一致 —— 速度错了接触力就不对，
            #    夹爪会以高速扫过试管而抓不住。
            _advance(frame_dt, per_frame_updates)

            # 3) 物体不手动设位姿 —— 由物理决定（见上方注释）。
            #    这里只做诊断：每 100 帧对比一次物理算出的物体位姿与录制值，
            #    用来判断「物理是否真的抓住了物体」。
            if frames_written % 100 == 0 and obj_views:
                for label, view in obj_views.items():
                    pos, quat = episode.objects[label]
                    if not np.all(np.isfinite(pos[i])):
                        continue
                    try:
                        cur = np.asarray(view.get_world_poses()[0]).reshape(-1)[:3]
                        dev = float(np.linalg.norm(cur - pos[i])) * 1000
                        _log(f"  [obj] {label} 与录制偏差 {dev:.1f} mm")
                    except Exception:  # noqa: BLE001
                        pass

            # 4) 回读校验（仅在关节帧有效时）
            frame_err = 0.0
            if verify_joints and values is not None:
                try:
                    actual = np.asarray(
                        art.get_dof_positions(dof_indices=target_idx), dtype=np.float32
                    ).reshape(-1)
                    frame_err = float(np.max(np.abs(actual - values.reshape(-1))))
                    errors_per_frame.append(frame_err)
                    max_err = max(max_err, frame_err)
                except Exception as exc:  # noqa: BLE001
                    # get_dof_positions 只支持 tensor 后端；未启用时这里会抛。
                    # 校验不是主流程，失败就关掉，不要中断渲染。
                    _log(f"WARN 关节回读不可用，关闭校验: {exc}")
                    verify_joints = False

            # 5) 渲染 + 采集
            #
            # 关键：pause_timeline 必须为 False。
            # 物体靠物理演化，若每帧渲染后把 timeline 暂停，物理就停住了，
            # 试管不会被夹爪带起来（之前的「试管不动」就是这么来的）。
            # delta_time=0.0 保证 timeline 不自行跳时间，演化步数完全由
            # 上面的 steps_per_frame × app.update() 决定。
            want_render = render_indices is None or i in render_indices
            if want_render:
                rep.orchestrator.step(delta_time=0.0, pause_timeline=False)
                info = {
                    "frame": i,
                    "t": float(episode.t[i]),
                    "joint_error": frame_err,
                }
                _collect_frame(out, i, cameras)
                frames_written += 1

            # 诊断：每 100 帧打印一次关节回读 + 物体偏差，
            # 用绝对帧号，避免与「只渲染部分帧」时的计数不一致。
            if i % 100 == 0:
                grip_val = float("nan")
                try:
                    back_all = np.asarray(art.get_dof_positions(), dtype=np.float32).reshape(-1)
                    if "left_gripper_joint" in dof_names:
                        grip_val = float(back_all[dof_names.index("left_gripper_joint")])
                except Exception:  # noqa: BLE001
                    pass
                obj_msg = ""
                for label, view in obj_views.items():
                    pos_o, _q = episode.objects[label]
                    if not np.all(np.isfinite(pos_o[i])):
                        continue
                    try:
                        cur = np.asarray(view.get_world_poses()[0]).reshape(-1)[:3]
                        dev = float(np.linalg.norm(cur - pos_o[i])) * 1000
                        obj_msg += f" {label}_dev={dev:.1f}mm"
                    except Exception:  # noqa: BLE001
                        pass
                _log(
                    f"  [diag] frame={i} left_grip={grip_val:+.4f} "
                    f"joint_err={frame_err * 1000:.2f}mrad{obj_msg}"
                )

            # 物理物体偏差统计：这是判断「物理是否真的抓住了物体」的核心指标。
            # 与录制位姿偏离越小，说明物理复现越忠实。
            if obj_views:
                for label, view in obj_views.items():
                    pos_o, _q = episode.objects[label]
                    if not np.all(np.isfinite(pos_o[i])):
                        continue
                    try:
                        cur = np.asarray(view.get_world_poses()[0]).reshape(-1)[:3]
                        dev = float(np.linalg.norm(cur - pos_o[i])) * 1000
                        obj_dev.setdefault(label, []).append(dev)
                    except Exception:  # noqa: BLE001
                        pass

            if on_frame is not None and want_render:
                on_frame(i, info)

            if (i % 200) == 0:
                _log(
                    f"  progress frame={i}/{n_end}  "
                    f"t={episode.t[i]:.2f}s  joint_err={frame_err * 1000:.2f} mrad"
                )
    finally:
        _close_cameras(cameras)

    wall = time.monotonic() - t_wall
    obj_stats = {
        k: {
            "mean_mm": float(np.mean(v)),
            "p95_mm": float(np.percentile(v, 95)),
            "max_mm": float(np.max(v)),
        }
        for k, v in obj_dev.items() if v
    }
    result = RenderResult(
        episode_path=str(episode_path),
        output_dir=str(out),
        frames_written=frames_written,
        cameras=[c["name"] for c in cameras],
        dof_names=dof_names,
        skipped_dofs=skipped,
        missing_joints=missing,
        joint_max_error=max_err,
        wall_seconds=wall,
        extras={
            "frame_range": [n_start, n_end],
            "joint_error_p95": (
                float(np.percentile(errors_per_frame, 95)) if errors_per_frame else None
            ),
            "steps_per_frame": steps_per_frame,
            "object_deviation": obj_stats,
        },
    )

    _log(
        f"完成: 渲染 {frames_written} 帧 / {wall:.1f}s "
        f"({frames_written / max(wall, 1e-6):.1f} fps)"
    )
    _log(f"关节跟踪 max_err = {max_err * 1000:.3f} mrad")
    if obj_stats:
        _log("物理物体与录制位姿的偏差（越小说明物理复现越忠实）:")
        for k, st in obj_stats.items():
            _log(
                f"  {k}: mean {st['mean_mm']:.1f} mm  "
                f"p95 {st['p95_mm']:.1f} mm  max {st['max_mm']:.1f} mm"
            )
        worst = max(st["p95_mm"] for st in obj_stats.values())
        if worst > 50.0:
            _log(
                "WARN 物体偏差偏大 (>50mm p95)。物理抓取可能没复现录制时的接触，"
                "检查初始位姿/摩擦系数，或增大 steps_per_frame。"
            )
    if verify_joints and max_err > 0.05:
        _log(
            "WARN 关节跟踪误差偏大 (>50 mrad)。相机数据可能与录制姿态不符，"
            "可尝试增大 steps_per_frame。"
        )

    if owns_app:
        app.close()
    return result


# ---------------------------------------------------------------------------
# 相机
# ---------------------------------------------------------------------------

def _setup_cameras(rep, task_config) -> list[dict]:
    """为 task 配置里的每个相机创建 render product + annotator。"""
    specs = getattr(task_config, "cameras", None) or []
    if not specs:
        _log("WARN task_config.cameras 为空")
        return []

    out_cams: list[dict] = []
    for spec in specs:
        name = spec.name
        width, height = (
            (int(spec.resolution[0]), int(spec.resolution[1]))
            if spec.resolution
            else (640, 480)
        )
        try:
            rp = rep.create.render_product(spec.prim_path, (width, height))
        except Exception as exc:  # noqa: BLE001
            _log(f"相机 {name}: 创建 render product 失败 ({spec.prim_path}): {exc}")
            continue
        rgb_annot = rep.annotators.get("rgb")
        depth_annot = rep.annotators.get("distance_to_image_plane")
        rgb_annot.attach(rp)
        depth_annot.attach(rp)
        out_cams.append({
            "name": name,
            "render_product": rp,
            "rgb": rgb_annot,
            "depth": depth_annot,
        })
        _log(f"相机 {name}: {width}x{height} @ {spec.prim_path}")
    return out_cams


def _collect_frame(out_dir: Path, frame_idx: int, cameras: list[dict]) -> None:
    """把当前帧的图像落盘。命名 frame_XXXXXX.npz 与图像分开存。"""
    import carb

    frame_dir = out_dir / "frames"
    frame_dir.mkdir(parents=True, exist_ok=True)
    payload: dict[str, Any] = {}
    for cam in cameras:
        name = cam["name"]
        rgb = _annot_data(cam["rgb"])
        depth = _annot_data(cam["depth"])
        if rgb is not None:
            payload[f"{name}_rgb"] = rgb
        if depth is not None:
            payload[f"{name}_depth"] = depth
    if payload:
        np.savez_compressed(frame_dir / f"frame_{frame_idx:06d}.npz", **payload)


def _annot_data(annot) -> np.ndarray | None:
    try:
        data = annot.get_data()
    except Exception:  # noqa: BLE001
        return None
    if data is None:
        return None
    arr = np.asarray(data)
    if arr.size == 0:
        return None
    return arr


def _close_cameras(cameras: list[dict]) -> None:
    for cam in cameras:
        rp = cam.get("render_product")
        if rp is None:
            continue
        try:
            rp.destroy()
        except Exception:  # noqa: BLE001
            pass
