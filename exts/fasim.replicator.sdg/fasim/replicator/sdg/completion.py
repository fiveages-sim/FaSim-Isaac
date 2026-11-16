"""任务完成检测 —— 采集期间判断「编排已经把东西放到位了」。

为什么需要
==========
半自动数采的循环是::

    随机化 → 开始采集 → 一键起编排 → 编排跑完归位 → 结束采集

如果不判断完成，「编排跑完」和「任务真的做成」是两件事：

- 编排超时/插偏了照样会往下走、照样回 HOME —— 采到的是**失败轨迹**，
  混进训练集会教坏策略。
- 人得盯着进度、手点结束 —— 一轮也省不下来。

所以这里在采集回调里旁路判断「试管是不是已经进了左侧架子的 hole_3」。

判据（G1D 试管任务专用，不做通用抽象）
======================================
三个条件同时满足：

1. **xy 已对准**：试管中心 与 ``hole_3`` 世界位置的水平距离 < ``xy_tol_m``
2. **z 已插入**：试管中心 比孔底高不超过 ``dz_inside_mm``
3. **姿态仍竖直**：试管轴与竖直方向夹角 < ``max_tilt_deg``

为什么必须有 z
--------------
只看 xy 会**大幅提前误判**。实测（run_001）：::

    帧 300  d_xy=12.2mm  dz=+46.3mm    ← 悬停在孔口上方对准
    帧 330  d_xy= 0.15mm dz=+39.8mm    ← 对准完成，还悬在空中
    帧 360  d_xy= 2.11mm dz=-15.1mm    ← 开始下探
    帧 380  d_xy= 3.26mm dz=-101.4mm   ← 真正插到底 ✅

``insert`` 技能的流程是「悬停孔口上方 → 空中对齐 xy → 再垂直下探」，
所以**悬空对准阶段 xy 就已经接近 0**。只有 xy 判据会在帧 305 就报完成，
比真实完成（376）早 71 帧（3.5s），而且失败轨迹也会被判成功。

z 的量纲：``hole_3`` 的局部 z=0.110 是**孔底**；试管中心在管身中部，
插到底时中心比孔底高约 98mm（41 run 实测 dz = -98 ~ -102mm）。
悬空对准时 dz ≈ +40mm。两者相差 140mm，用 -60mm 作分界极稳。

关于 xy 阈值（重要，踩过一次弯路）
----------------------------------
第一版把 xy 阈值设成 20mm，理由是「41 条里 39 条 ≤20mm」。**这是错的**：

- 41 条全部倾角 ≤13.6°、dz 全部 -98~-102mm —— **都真的插进去了**。
- dxy 的分布是**连续的**：0.65 → 21.82mm，最大相邻间隔只有 2.01mm，
  **没有任何自然分界**。20mm 纯粹是从分布尾部硬砍的，没有物理意义。
- 那两条被砍掉的（21.82 / 20.67mm）与其余数据同量级，是同一个孔里的
  不同落点（孔本身有宽度，试管 Ø13mm 插进孔里允许偏移）。

真正的「插失败」长什么样（不在本数据集，但必须能识别）：

- **没插进去**：试管横躺在架面上 → dz ≈ 0 或为正，倾角接近 90°
- **碰倒了**：倾角 > 45°
- **掉在孔边**：dz 明显大于 -60mm

所以 xy 阈值的作用不是「判断偏多少算失败」，而是「防止把**别的孔**
或**架外**当成目标孔」。孔间距约 30mm、架长 200mm，取 25mm 合理：
既能容忍真实落点分散（最大 21.82mm），又不会跨到相邻孔。

``max_tilt_deg`` 才是识别「插歪/碰倒」的判据：正常插入 ≤13.6°，
碰倒 >45°，取 30° 分界。

阈值实测（41 个 run，全部真实插入成功）
--------------------------------------
::

    dxy:  0.65 ~ 21.82mm   （连续分布，无自然分界）
    dz:   -98 ~ -102mm     （全部到位）
    tilt: 0.85 ~ 13.62°    （全部接近竖直）

用 ``xy<25mm 且 dz<-60mm 且 tilt<30°`` → **41/41 全部判定成功**。

随机化适配
----------
架子被随机平移后 ``rack`` 的世界位姿变了，所以 **hole_3 的世界位置必须
每帧现算**，不能用固定常量::

    hole_world = rack_pos + R(rack_quat) @ hole_local

这正是 ``fasim/replicator/sdg/randomize.py`` 里
``get_subframe_world_pose()`` 做的事，这里直接复用它的数学，
但**不读 USD**（回调里不能走 ROS 服务，见 live_capture 的注释）。

为什么不用 ``get_subframe_world_pose()`` 直接读
---------------------------------------------
那个函数要从 USD 读 ``hole_3`` 的局部 TRS（走 ``/get_prim_attribute``），
在 ``app.update()`` 回调里会超时。而 hole_3 的局部 TRS 是**资产常量**
（不随随机化变），所以只需要在 arm_capture 时读一次、缓存起来，
之后每帧只做一次旋转+平移，纯 numpy，微秒级。

编排侧同源校验：``/get_prim_attribute`` 对
``/World/room/scene/test_tube_rack/hole_3`` 实测返回
``(-0.021000, -0.010000, 0.110000)``（见运行日志 [ObjectBind] 行）。
"""

from __future__ import annotations

import numpy as np

#: hole_3 在 test_tube_rack 局部系的 translate（米）。
#:
#: 来源：编排 insert 技能的 [ObjectBind] 日志实测值。这是**资产常量**，
#: 随机化不会改它（随机化只改 rack 的整体世界位姿）。
#: 若换孔/换架子，改这里或走 ``done_when.hole_offset`` 配置覆盖。
#:
#: 准确性已验证：41 个 run 的试管末位在 rack 局部系的均值与这个值只差
#: ``(2.42, -1.69) mm`` —— 说明常量本身没系统偏差，落点分散是真实的插偏。
HOLE_3_LOCAL = (-0.021, -0.010, 0.110)

#: 默认 xy 距离阈值（米）。
#:
#: **不是**「偏多少算失败」的分界（实测 dxy 连续分布 0.65~21.82mm，无自然
#: 分界），而是「防止把别的孔或架外当成目标孔」—— 孔间距约 30mm，
#: 架长 200mm，取 25mm 既能容纳真实落点分散，又不会跨到相邻孔。
DEFAULT_XY_TOL_M = 0.025

#: dz 上限（毫米）：试管中心相对孔底的高度。
#:
#: 插到底时 dz ≈ -98 ~ -102mm；悬空对准时 ≈ +40mm。
#: 取 -60mm 作分界（离两者各留 40mm 余量）。
DEFAULT_DZ_INSIDE_MM = -60.0

#: 试管轴与竖直方向的最大允许夹角（度）。
#:
#: 这才是识别「插歪 / 碰倒」的判据：41 个成功 run 实测 0.85~13.62°，
#: 而碰倒/横躺会 >45°。取 30° 分界。
DEFAULT_MAX_TILT_DEG = 30.0


def _quat_rotate_wxyz(q, v) -> np.ndarray:
    """wxyz 四元数旋转向量（与 Isaac/UsdGeom 同约定）。"""
    w, x, y, z = (float(t) for t in np.asarray(q).reshape(-1)[:4])
    vx, vy, vz = (float(t) for t in np.asarray(v).reshape(-1)[:3])
    tx = 2.0 * (y * vz - z * vy)
    ty = 2.0 * (z * vx - x * vz)
    tz = 2.0 * (x * vy - y * vx)
    return np.array([
        vx + w * tx + (y * tz - z * ty),
        vy + w * ty + (z * tx - x * tz),
        vz + w * tz + (x * ty - y * tx),
    ])


def tilt_deg_from_upright(quat_wxyz) -> float:
    """物体局部 +Z 轴与**世界竖直**方向的夹角（度）。

    试管在资产里是沿 +Z 竖立的，所以「+Z 偏离竖直多少度」就是它的倾倒角。
    正常插入 ≤13.6°；碰倒/横躺 >45°。
    """
    axis = _quat_rotate_wxyz(quat_wxyz, np.array([0.0, 0.0, 1.0]))
    cos = float(np.clip(axis[2], -1.0, 1.0))   # 与世界 +Z 的夹角
    return float(np.degrees(np.arccos(cos)))


def load_config(task_config) -> dict | None:
    """从任务配置取 ``done_when`` 段；没有就返回 None（不启用完成检测）。"""
    if task_config is None:
        return None
    cfg = getattr(task_config, "done_when", None)
    if cfg is None and isinstance(task_config, dict):
        cfg = task_config.get("done_when")
    if not cfg:
        return None
    return dict(cfg)


def resolve_hole_offset(cfg: dict) -> np.ndarray:
    """孔位的局部偏移：配置优先，否则用实测的 hole_3 常量。"""
    raw = cfg.get("hole_offset")
    if raw is None:
        return np.asarray(HOLE_3_LOCAL, dtype=np.float64)
    arr = np.asarray(raw, dtype=np.float64).reshape(-1)
    if arr.size != 3:
        raise ValueError(f"done_when.hole_offset 需要 3 个数，得到 {raw!r}")
    return arr


class CompletionChecker:
    """逐帧判断任务是否完成。

    用法（在 ``live_capture.arm_capture`` 里建、在 ``_emit_frame`` 里喂）::

        checker = CompletionChecker(cfg)          # cfg = done_when 段
        ...
        verdict = checker.update(payload)         # 每帧一次
        if verdict is not None:
            # verdict = {"reason": ..., "d_xy_mm": ..., ...}
            ...
    """

    def __init__(self, cfg: dict):
        self.cfg = dict(cfg or {})
        self.object_label = str(self.cfg.get("object") or "test_tube")
        self.rack_label = str(self.cfg.get("rack") or "test_tube_rack")
        self.tol_m = float(self.cfg.get("xy_tol_m", DEFAULT_XY_TOL_M))
        # z 判据：试管中心相对孔底的高度上限（mm）。
        # 悬空对准 ≈ +40mm，插到底 ≈ -100mm，取 -60mm 分界。
        # 设为 None 可关掉。
        raw_dz = self.cfg.get("dz_inside_mm", DEFAULT_DZ_INSIDE_MM)
        self.dz_inside_mm = None if raw_dz is None else float(raw_dz)
        # 姿态判据：这才是识别「插歪/碰倒」的判据（正常 ≤13.6°，碰倒 >45°）。
        raw_tilt = self.cfg.get("max_tilt_deg", DEFAULT_MAX_TILT_DEG)
        self.max_tilt_deg = None if raw_tilt is None else float(raw_tilt)
        self.hole_offset = resolve_hole_offset(self.cfg)
        # 连续多少帧满足才算（防抖：试管经过孔口上方时会瞬间重合）
        self.min_consecutive = max(1, int(self.cfg.get("consecutive_frames", 8)))
        # 完成后是否立即停；False 时只返回判定，由外部决定（默认 True）
        self.stop_on_complete = bool(self.cfg.get("stop_on_complete", True))

        self._streak = 0
        self._done = False
        self._last_d_xy_mm: float | None = None
        self._last_dz_mm: float | None = None
        self._last_tilt_deg: float | None = None
        self._done_frame: int | None = None

    def update(self, payload: dict) -> dict | None:
        """喂一帧。完成时返回判定 dict，否则返回 None（之后不再返回）。"""
        if self._done:
            return None

        op = payload.get(f"obj/{self.object_label}_pos")
        rp = payload.get(f"obj/{self.rack_label}_pos")
        rq = payload.get(f"obj/{self.rack_label}_quat")
        if op is None or rp is None or rq is None:
            # 缺字段：不判（可能配置写错 label，或该任务没采物体位姿）
            self._streak = 0
            return None

        tube = np.asarray(op, dtype=np.float64).reshape(-1)[:3]
        rack_pos = np.asarray(rp, dtype=np.float64).reshape(-1)[:3]
        rack_quat = np.asarray(rq, dtype=np.float64).reshape(-1)[:4]

        # 随机化后架子会动 → hole 世界位置每帧现算
        hole = rack_pos + _quat_rotate_wxyz(rack_quat, self.hole_offset)
        d_xy = float(np.linalg.norm(tube[:2] - hole[:2]))
        dz_mm = float((tube[2] - hole[2]) * 1000.0)
        self._last_d_xy_mm = d_xy * 1000.0
        self._last_dz_mm = dz_mm

        # 姿态：需要 test_tube_quat。没采就跳过这条判据（不因此判失败）。
        tq = payload.get(f"obj/{self.object_label}_quat")
        tilt = None
        if tq is not None:
            tilt = tilt_deg_from_upright(np.asarray(tq, dtype=np.float64).reshape(-1)[:4])
            self._last_tilt_deg = tilt

        xy_ok = d_xy <= self.tol_m
        z_ok = (self.dz_inside_mm is None) or (dz_mm <= self.dz_inside_mm)
        tilt_ok = (
            self.max_tilt_deg is None
            or tilt is None
            or tilt <= self.max_tilt_deg
        )

        if xy_ok and z_ok and tilt_ok:
            self._streak += 1
        else:
            self._streak = 0

        if self._streak < self.min_consecutive:
            return None

        self._done = True
        self._done_frame = int(payload.get("frame", -1))
        return {
            "reason": "task_completed",
            "object": self.object_label,
            "rack": self.rack_label,
            "d_xy_mm": round(self._last_d_xy_mm, 2),
            "dz_mm": round(dz_mm, 2),
            "tilt_deg": None if tilt is None else round(tilt, 2),
            "xy_tol_m": self.tol_m,
            "dz_inside_mm": self.dz_inside_mm,
            "max_tilt_deg": self.max_tilt_deg,
            "consecutive_frames": self._streak,
            "frame": self._done_frame,
        }

    def status(self) -> dict:
        """当前状态（供 socket status 查询/日志）。"""
        return {
            "enabled": True,
            "object": self.object_label,
            "rack": self.rack_label,
            "xy_tol_mm": round(self.tol_m * 1000.0, 2),
            "dz_inside_mm": self.dz_inside_mm,
            "max_tilt_deg": self.max_tilt_deg,
            "consecutive_frames": self.min_consecutive,
            "streak": self._streak,
            "done": self._done,
            "done_frame": self._done_frame,
            "last_d_xy_mm": (
                round(self._last_d_xy_mm, 2)
                if self._last_d_xy_mm is not None else None
            ),
            "last_dz_mm": (
                round(self._last_dz_mm, 2)
                if self._last_dz_mm is not None else None
            ),
            "last_tilt_deg": (
                round(self._last_tilt_deg, 2)
                if self._last_tilt_deg is not None else None
            ),
        }
