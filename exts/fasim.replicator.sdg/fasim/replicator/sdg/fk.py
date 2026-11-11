"""从关节角算 tcp 位姿 —— 纯 numpy，不依赖 pinocchio。

用途（只有两个，都在"指令"侧）
================================
1. **末端指令位姿**（主要用途）
   EE 回放必须发**指令目标**而不是实测值，否则运控二次叠加稳态误差::

       编排:   目标 → 运控 → 实测 = 目标 − 7~9mm      ✅
       EE回放: 实测 → 运控 → 实测 − 7~9mm            ❌ 稳定偏下

   指令位姿只存在于运控 MPC 内部；Isaac 侧只有关节指令角
   ``joint_pos_cmd``（``get_dof_position_targets``，实测与 actual 无时间
   滞后、连续可导，可信）。所以用 FK 把关节指令角换算成末端指令位姿。

2. **末端真值的离线交叉验证**
   采集侧的真值已经用「父刚体 + 固定偏移」直读 PhysX（见 ``live_capture``），
   本模块只用来对拍（差异应 < 3mm）。

**为什么末端真值不用 FK**：``left_tcp`` 是纯 Xform，newton kit 关了
PhysX→USD/Fabric 同步（``updateToUsd=false``），直接读只会得到静态值。
刚体读取绕开了这个管道，比 FK 更贴近"仿真实际发生的事"。

为什么不用 pinocchio
====================
项目原则是不引入外部工具。而且 Isaac python 与 ``sdg.sh pack`` 用的系统
python 里都没有 pinocchio。planning URDF 结构简单（只有 fixed / revolute /
prismatic），链式变换用 numpy 累乘即可。

实现要点
========
- URDF ``<origin xyz rpy>`` 是 parent→child 固定变换，固定轴 XYZ：
  ``R = Rz(yaw) @ Ry(pitch) @ Rx(roll)``
- 关节运动叠在 origin 之后（joint frame 内）：
  ``T = Trans(xyz) @ R_rpy @ Rot(axis, q)``
- 只走 root→tip 的单链，忽略其他分支（脚轮/相机等）

迁移到新机器人
==============
**不需要改本文件**。在 ``configs/<robot>/robot.yaml`` 配::

    planning:
      urdf: auto            # 从运控缓存自动找（改配置会生成新 hash，取最新）
      cache_dir: ""         # 留空 = $OCS2_PLANNING_URDF_DIR 或 /tmp
      ee_frames:
        left: left_tcp      # 对应运控 .info 的 eeFrame
        right: right_tcp    # 对应 eeFrame1

前置条件：运控已经在跑过（缓存目录里得有
``<cache>/ocs2_ros2/<robot>/planning_urdf/*.urdf``）。URDF 的缓存规则见
robot_common_launch/common/robot_utils.py 的 ``get_planning_urdf_cache_dir``
与 ``_planning_urdf_cache_key`` —— hash 依赖 xacro 映射参数，无法复算，
所以按 mtime 取最新。

若某个机器人不想启用 FK：``urdf: none``，此后 EE 回放回退发实测位姿
（精度差 7~9mm）。指定了 ``robot_key`` 却找不到对应 URDF 时**不会**跨机器人
回退 —— 宁可失败也不要拿错模型算出错误位姿。
"""

from __future__ import annotations

import math
import os
import xml.etree.ElementTree as ET
from pathlib import Path
from typing import Any

import numpy as np

#: 兜底：手工 xacro 生成的 URDF（语义相同，但可能不是运控当前用的版本）
FALLBACK_URDFS = (
    "/tmp/g1d_urdf/g1d.urdf",
    "~/g1d_urdf/g1d.urdf",
)

#: 默认末端 frame 名（与 ``mobile_base_tcp.info`` 的 eeFrame/eeFrame1 一致）
DEFAULT_EE_FRAMES = {"left": "left_tcp", "right": "right_tcp"}


def _log(msg: str) -> None:
    print(f"[fk] {msg}", flush=True)


def find_planning_urdf(
    robot_key: str | None = None,
    cache_dir: str | Path | None = None,
) -> Path | None:
    """找 OCS2 实际加载的 planning URDF。

    运控把（xacro 参数 + schema 版本）的 sha256[:16] 当文件名，缓存到::

        $OCS2_PLANNING_URDF_DIR/<robot>/planning_urdf/<hash>.urdf
        （未设该环境变量时 base 为 /tmp）

    见 robot_common_launch/common/robot_utils.py 的 get_planning_urdf_cache_dir
    与 _planning_urdf_cache_key。hash 依赖 xacro 映射参数，我们无法复算，所以
    **取 mtime 最新那份** —— 改过配置会生成新 hash，最新的就是当前在用的。

    Args:
        robot_key: 机器人名（缓存目录名）。给定时只在该子目录找。
        cache_dir: 缓存根目录；``None`` 时读 ``OCS2_PLANNING_URDF_DIR`` 或 ``/tmp``。

    Returns:
        URDF 路径；找不到返回 ``None``。
    """
    candidates: list[Path] = []
    bases: list[Path] = []
    if cache_dir:
        bases.append(Path(str(cache_dir)).expanduser())
    env_dir = os.environ.get("OCS2_PLANNING_URDF_DIR", "").strip()
    if env_dir:
        bases.append(Path(env_dir))
    bases.append(Path("/tmp"))

    for base in bases:
        if not base.is_dir():
            continue
        # 根目录可能直接是 <...>/planning_urdf，也可能是 <...>/ocs2_ros2
        for pat in (
            "ocs2_ros2/*/planning_urdf/*.urdf",
            "*/planning_urdf/*.urdf",
            "planning_urdf/*.urdf",
        ):
            hits = [p for p in base.glob(pat) if p.is_file()]
            if robot_key:
                hits = [p for p in hits if robot_key in p.parts]
            candidates.extend(hits)
        if candidates:
            break

    if candidates:
        return max(candidates, key=lambda p: p.stat().st_mtime)
    if robot_key:
        # 指定了机器人却找不到 —— **不要**跨机器人回退，否则会拿错 URDF
        # 算出完全错误的末端位姿（多机器人场景下尤其危险）。
        return None
    for pat in FALLBACK_URDFS:
        p = Path(pat).expanduser()
        if p.is_file():
            return p
    return None


def resolve_urdf_from_config(planning: dict[str, Any] | None,
                             robot_key: str | None = None) -> Path | None:
    """按 ``robot.yaml`` 的 ``planning`` 段决定用哪份 URDF。

    ``urdf`` 取值：
      - ``"auto"``（默认）：从运控缓存自动找
      - ``"none"``：不启用 FK（返回 None）
      - 其它：当路径用

    迁移新机器人时只需要改这段配置 —— 见 ``configs/*/robot.yaml`` 的说明。
    """
    cfg = dict(planning or {})
    raw = str(cfg.get("urdf") or "auto").strip()
    if raw.lower() in ("none", "off", "disabled", ""):
        return None
    if raw.lower() not in ("auto", "cached", "cache"):
        p = Path(raw).expanduser()
        if p.is_file():
            return p
        _log(f"WARN planning.urdf 指定的文件不存在: {p}（回退自动查找）")
    return find_planning_urdf(robot_key=robot_key,
                              cache_dir=cfg.get("cache_dir") or None)


def make_fk(planning: dict[str, Any] | None = None,
            robot_key: str | None = None) -> "TcpFK | None":
    """按配置构造 ``TcpFK``；未启用或无 URDF 时返回 ``None``（不抛异常）。

    采集/转换链路用这个，保证「没配 FK 也不影响其他功能」。
    """
    cfg = dict(planning or {})
    urdf = resolve_urdf_from_config(cfg, robot_key)
    if urdf is None:
        return None
    frames = cfg.get("ee_frames") or None
    try:
        return TcpFK(urdf, ee_frames=frames or None)
    except Exception as exc:  # noqa: BLE001
        _log(f"WARN 构造 FK 失败（{urdf}）: {exc}")
        return None
    for pat in FALLBACK_URDFS:
        p = Path(pat)
        if p.is_file():
            return p
    return None


# ---------------------------------------------------------------- 数学工具


def rpy_to_matrix(roll: float, pitch: float, yaw: float) -> np.ndarray:
    """URDF 的固定轴 XYZ rpy → 旋转矩阵：R = Rz(yaw) Ry(pitch) Rx(roll)。"""
    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)
    return np.array([
        [cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
        [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
        [-sp, cp * sr, cp * cr],
    ])


def axis_angle_to_matrix(axis: np.ndarray, angle: float) -> np.ndarray:
    """Rodrigues：绕单位轴 axis 转 angle（弧度）→ 旋转矩阵。"""
    a = np.asarray(axis, dtype=np.float64).reshape(3)
    n = float(np.linalg.norm(a))
    if n < 1e-12 or abs(angle) < 1e-12:
        return np.eye(3)
    x, y, z = a / n
    c, s = math.cos(angle), math.sin(angle)
    C = 1.0 - c
    return np.array([
        [c + x * x * C, x * y * C - z * s, x * z * C + y * s],
        [y * x * C + z * s, c + y * y * C, y * z * C - x * s],
        [z * x * C - y * s, z * y * C + x * s, c + z * z * C],
    ])


def _homogeneous(R: np.ndarray, t: np.ndarray) -> np.ndarray:
    T = np.eye(4, dtype=np.float64)
    T[:3, :3] = R
    T[:3, 3] = np.asarray(t, dtype=np.float64).reshape(3)
    return T


def _parse_vec(text: str | None, default: tuple[float, ...] = (0.0, 0.0, 0.0)):
    if not text:
        return np.array(default, dtype=np.float64)
    vals = [float(x) for x in str(text).replace(",", " ").split()]
    out = np.zeros(len(default), dtype=np.float64)
    out[: len(vals)] = vals[: len(default)]
    return out


# ---------------------------------------------------------------- URDF 链


class _Joint:
    """URDF 里的一个关节（只保留 FK 需要的字段）。"""

    __slots__ = ("name", "type", "parent", "child", "xyz", "rpy", "axis")

    def __init__(self, el: ET.Element) -> None:
        self.name = el.get("name") or ""
        self.type = el.get("type") or "fixed"
        self.parent = (el.find("parent").get("link") if el.find("parent") is not None else "") or ""
        self.child = (el.find("child").get("link") if el.find("child") is not None else "") or ""
        o = el.find("origin")
        self.xyz = _parse_vec(o.get("xyz") if o is not None else None)
        rpy = _parse_vec(o.get("rpy") if o is not None else None)
        self.rpy = (float(rpy[0]), float(rpy[1]), float(rpy[2]))
        ax = el.find("axis")
        self.axis = _parse_vec(ax.get("xyz") if ax is not None else None, (0.0, 0.0, 1.0))

    @property
    def is_actuated(self) -> bool:
        return self.type in ("revolute", "continuous", "prismatic")

    def fixed_transform(self) -> np.ndarray:
        return _homogeneous(rpy_to_matrix(*self.rpy), self.xyz)

    def motion(self, q: float) -> np.ndarray:
        """关节运动（在 joint frame 里）。fixed 关节恒为单位阵。"""
        if self.type in ("revolute", "continuous"):
            return _homogeneous(axis_angle_to_matrix(self.axis, q), np.zeros(3))
        if self.type == "prismatic":
            a = np.asarray(self.axis, dtype=np.float64).reshape(3)
            n = float(np.linalg.norm(a))
            return _homogeneous(np.eye(3), (a / n if n > 1e-12 else a) * q)
        return np.eye(4)


def _load_urdf(urdf_path: Path) -> tuple[dict[str, list[_Joint]], set[str]]:
    """解析 URDF → ``(parent_link → [joint...], 有父关节的 link 集合)``。"""
    root = ET.parse(str(urdf_path)).getroot()
    by_parent: dict[str, list[_Joint]] = {}
    has_parent: set[str] = set()
    for el in root.findall("joint"):
        j = _Joint(el)
        by_parent.setdefault(j.parent, []).append(j)
        has_parent.add(j.child)
    return by_parent, has_parent


def _find_chain(
    by_parent: dict[str, list[_Joint]], has_parent: set[str], tip: str
) -> list[_Joint]:
    """从根 link 到 ``tip`` 的单链（深度优先）。

    Args:
        by_parent: 解析结果。
        has_parent: 有父关节的 link（用来找根）。
        tip: 目标 link 名。

    Raises:
        KeyError: 找不到该 tip。
    """
    roots = [l for l in by_parent if l not in has_parent]
    if not roots:
        raise KeyError(f"URDF 里找不到根 link（{len(by_parent)} 个 parent）")
    root = roots[0]

    def dfs(link: str, acc: list[_Joint], seen: set[str]):
        if link == tip:
            return list(acc)
        if link in seen:  # 防环
            return None
        seen.add(link)
        for j in by_parent.get(link, ()):
            acc.append(j)
            got = dfs(j.child, acc, seen)
            if got is not None:
                return got
            acc.pop()
        return None

    chain = dfs(root, [], set())
    if chain is None:
        raise KeyError(f"URDF 里找不到 link {tip!r}（根 {root!r}）")
    return chain


class TcpFK:
    """从 18 个关节角算左右 tcp 位姿。纯 numpy。

    Args:
        urdf_path: planning URDF；``None`` → 自动找。
        ee_frames: ``{side: link_name}``，默认 left_tcp / right_tcp。

    Attributes:
        joint_order: 驱动关节顺序（URDF 遍历序）。**必须按名字对齐** ——
            articulation 的 DOF 顺序与 /joint_states、URDF 都不同。
    """

    def __init__(
        self,
        urdf_path: str | Path | None = None,
        ee_frames: dict[str, str] | None = None,
    ) -> None:
        p = Path(urdf_path) if urdf_path else find_planning_urdf()
        if p is None or not Path(p).is_file():
            raise FileNotFoundError(
                "找不到 planning URDF。先跑一次运控生成 "
                "/tmp/ocs2_ros2/<robot>/planning_urdf/*.urdf，"
                "或用 xacro 生成后显式传 urdf_path"
            )
        self.urdf_path = Path(p)
        self.ee_frames = dict(ee_frames or DEFAULT_EE_FRAMES)

        self._by_parent, self._has_parent = _load_urdf(self.urdf_path)

        # 每个关节链里"会被 q 驱动"的那些，按链顺序记下来，同时确定全局
        # 关节顺序（与 pinocchio 的 model.names[1:] 一致：按树遍历序）。
        self._chains: dict[str, list[tuple[_Joint, int]]] = {}
        seen_active: list[str] = []
        for side, tip in self.ee_frames.items():
            chain = _find_chain(self._by_parent, self._has_parent, tip)
            marked: list[tuple[_Joint, int]] = []
            for j in chain:
                if j.is_actuated:
                    if j.name not in seen_active:
                        seen_active.append(j.name)
                    marked.append((j, seen_active.index(j.name)))
                else:
                    marked.append((j, -1))
            self._chains[side] = marked
        self.joint_order: list[str] = seen_active

        #: ``world_from_base``，由 ``solve_base_from_rest`` 填。
        self.base_R: np.ndarray | None = None
        self.base_t: np.ndarray | None = None

    # ------------------------------------------------------------ 映射

    def map_from_joint_names(self, joint_names: list[str]) -> np.ndarray:
        """数据里的关节名顺序 → URDF 关节顺序的取列索引。"""
        names = [str(n) for n in joint_names]
        missing = [n for n in self.joint_order if n not in names]
        if missing:
            raise ValueError(f"数据里缺少 URDF 关节 {missing}")
        return np.array([names.index(n) for n in self.joint_order], dtype=np.int64)

    # ------------------------------------------------------------ FK

    def _chain_fk(self, side: str, q18: np.ndarray) -> np.ndarray:
        """base→tcp 的 4x4 变换。"""
        T = np.eye(4, dtype=np.float64)
        for j, qi in self._chains[side]:
            T = T @ j.fixed_transform()
            if qi >= 0:
                T = T @ j.motion(float(q18[qi]))
        return T

    def tcp_poses(
        self,
        joint_pos: np.ndarray,
        joint_names: list[str],
        to_world: bool = True,
    ) -> dict[str, tuple[np.ndarray, np.ndarray]]:
        """整段轨迹的 tcp 位姿。

        Args:
            joint_pos: ``(N, J)`` 关节角。
            joint_names: 对应 J 列的名字。
            to_world: 换到 world 系（需先 ``solve_base_from_rest``）。

        Returns:
            ``{side: (pos (N,3), quat_wxyz (N,4))}``。
        """
        q = np.asarray(joint_pos, dtype=np.float64)
        cols = self.map_from_joint_names(joint_names)
        n = q.shape[0]
        out: dict[str, tuple[np.ndarray, np.ndarray]] = {
            s: (np.full((n, 3), np.nan), np.full((n, 4), np.nan))
            for s in self.ee_frames
        }
        T_wb = None
        if to_world and self.base_R is not None:
            T_wb = _homogeneous(self.base_R, self.base_t)

        for i in range(n):
            row = q[i][cols]
            if not np.all(np.isfinite(row)):
                continue
            for side in self.ee_frames:
                T = self._chain_fk(side, row)
                if T_wb is not None:
                    T = T_wb @ T
                out[side][0][i] = T[:3, 3]
                out[side][1][i] = _rot_to_wxyz(T[:3, :3])
        return out

    def tcp_pose_at(
        self, q_by_name: dict[str, float], to_world: bool = True
    ) -> dict[str, tuple[np.ndarray, np.ndarray]]:
        """单帧（按名字给关节角）。"""
        row = np.array([float(q_by_name[n]) for n in self.joint_order])
        res = {}
        T_wb = None
        if to_world and self.base_R is not None:
            T_wb = _homogeneous(self.base_R, self.base_t)
        for side in self.ee_frames:
            T = self._chain_fk(side, row)
            if T_wb is not None:
                T = T_wb @ T
            res[side] = (T[:3, 3], _rot_to_wxyz(T[:3, :3]))
        return res

    # ------------------------------------------------------------ base 反解

    def solve_base_from_rest(
        self,
        rest_pos: dict[str, np.ndarray],
        rest_quat: dict[str, np.ndarray] | None = None,
    ) -> tuple[np.ndarray, np.ndarray]:
        """用静止姿态（q≈0）的 tcp 世界位姿反解 ``world_from_base``。

        机器人上电时关节角全 ~0，tcp 相对 base 是固定几何量，所以一帧静止
        姿态就够。左右手各解一次再平均，抵消单侧噪声。

        Returns:
            ``(base_t, base_R)``，同时写回 ``self``。
        """
        q0 = np.zeros(len(self.joint_order))
        fk0 = {}
        for side in self.ee_frames:
            T = self._chain_fk(side, q0)
            fk0[side] = (T[:3, 3], _rot_to_wxyz(T[:3, :3]))

        R_wb = None
        if rest_quat:
            Rs = []
            for side, quat in rest_quat.items():
                if side not in fk0:
                    continue
                R_rec = _wxyz_to_rot(np.asarray(quat).reshape(-1)[:4])
                R_fk = _wxyz_to_rot(fk0[side][1])
                Rs.append(R_rec @ R_fk.T)
            if Rs:
                R_wb = _average_rotation(Rs)

        if R_wb is None and {"left", "right"} <= set(rest_pos):
            # 没姿态信息：用左右手连线定 yaw（G1D 平地，roll/pitch≈0）
            dw = np.asarray(rest_pos["right"])[:2] - np.asarray(rest_pos["left"])[:2]
            db = fk0["right"][0][:2] - fk0["left"][0][:2]
            yaw = float(np.arctan2(dw[1], dw[0]) - np.arctan2(db[1], db[0]))
            c, s = math.cos(yaw), math.sin(yaw)
            R_wb = np.array([[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]])
        if R_wb is None:
            R_wb = np.eye(3)

        diffs = [
            np.asarray(p, dtype=np.float64).reshape(-1)[:3] - R_wb @ fk0[s][0]
            for s, p in rest_pos.items()
            if s in fk0
        ]
        if not diffs:
            raise ValueError("rest_pos 里没有和 ee_frames 对应的手")
        self.base_R = R_wb
        self.base_t = np.mean(np.stack(diffs), axis=0)
        return self.base_t, self.base_R

    # ------------------------------------------------------------ 自检

    def validate_against(
        self,
        rec_pos: dict[str, np.ndarray],
        rec_quat: dict[str, np.ndarray],
        joint_pos: np.ndarray,
        joint_names: list[str],
        max_mm: float = 5.0,
    ) -> dict[str, Any]:
        """核对录制的末端位姿是否真是动态值（采集链路体检）。"""
        fk = self.tcp_poses(joint_pos, joint_names, to_world=True)
        per: dict[str, Any] = {}
        ok = True
        reasons: list[str] = []
        for side in self.ee_frames:
            if side not in rec_pos:
                continue
            rec = np.asarray(rec_pos[side], dtype=np.float64)
            fkp = fk[side][0]
            good = np.isfinite(rec).all(axis=1) & np.isfinite(fkp).all(axis=1)
            if not good.any():
                per[side] = {"error": "没有有效帧"}
                ok = False
                reasons.append(f"{side}: 无有效帧")
                continue
            d = np.linalg.norm(rec[good] - fkp[good], axis=1) * 1000.0
            r_ok = rec[np.isfinite(rec).all(axis=1)]
            rec_travel = (float(np.linalg.norm(r_ok - r_ok[0], axis=1).max() * 1000.0)
                          if len(r_ok) else 0.0)
            fk_travel = float(np.linalg.norm(fkp[good] - fkp[good][0], axis=1).max() * 1000.0)
            per[side] = {
                "mean_mm": round(float(d.mean()), 1),
                "max_mm": round(float(d.max()), 1),
                "recorded_travel_mm": round(rec_travel, 1),
                "fk_travel_mm": round(fk_travel, 1),
            }
            if d.mean() > max_mm:
                ok = False
                reasons.append(f"{side}: 与 FK 差 {d.mean():.0f}mm")
            if fk_travel > 50.0 and rec_travel < 1.0:
                ok = False
                reasons.append(
                    f"{side}: 录制值恒定为静态值（FK 动了 {fk_travel:.0f}mm）"
                )
        return {"ok": ok, "per_side": per, "reason": "; ".join(reasons) or "正常"}


# ---------------------------------------------------------------- 四元数


def _wxyz_to_rot(q: np.ndarray) -> np.ndarray:
    w, x, y, z = (float(v) for v in np.asarray(q).reshape(-1)[:4])
    n = w * w + x * x + y * y + z * z
    if n <= 0:
        return np.eye(3)
    s = 2.0 / n
    return np.array([
        [1 - s * (y * y + z * z), s * (x * y - z * w), s * (x * z + y * w)],
        [s * (x * y + z * w), 1 - s * (x * x + z * z), s * (y * z - x * w)],
        [s * (x * z - y * w), s * (y * z + x * w), 1 - s * (x * x + y * y)],
    ])


def _rot_to_wxyz(R: np.ndarray) -> np.ndarray:
    R = np.asarray(R, dtype=np.float64)
    tr = R[0, 0] + R[1, 1] + R[2, 2]
    if tr > 0:
        s = math.sqrt(tr + 1.0) * 2
        w, x = 0.25 * s, (R[2, 1] - R[1, 2]) / s
        y, z = (R[0, 2] - R[2, 0]) / s, (R[1, 0] - R[0, 1]) / s
    elif R[0, 0] > R[1, 1] and R[0, 0] > R[2, 2]:
        s = math.sqrt(1.0 + R[0, 0] - R[1, 1] - R[2, 2]) * 2
        w, x = (R[2, 1] - R[1, 2]) / s, 0.25 * s
        y, z = (R[0, 1] + R[1, 0]) / s, (R[0, 2] + R[2, 0]) / s
    elif R[1, 1] > R[2, 2]:
        s = math.sqrt(1.0 + R[1, 1] - R[0, 0] - R[2, 2]) * 2
        w, x = (R[0, 2] - R[2, 0]) / s, (R[0, 1] + R[1, 0]) / s
        y, z = 0.25 * s, (R[1, 2] + R[2, 1]) / s
    else:
        s = math.sqrt(1.0 + R[2, 2] - R[0, 0] - R[1, 1]) * 2
        w, x = (R[1, 0] - R[0, 1]) / s, (R[0, 2] + R[2, 0]) / s
        y, z = (R[1, 2] + R[2, 1]) / s, 0.25 * s
    q = np.array([w, x, y, z], dtype=np.float64)
    return q / np.linalg.norm(q)


def _average_rotation(Rs: list[np.ndarray]) -> np.ndarray:
    M = np.mean(np.stack(Rs), axis=0)
    U, _S, Vt = np.linalg.svd(M)
    R = U @ Vt
    if np.linalg.det(R) < 0:
        U[:, -1] *= -1
        R = U @ Vt
    return R


# ---------------------------------------------------------------- 离线入口


def analyze_episode(path: str | Path, urdf_path: str | Path | None = None) -> dict:
    """离线体检一个 episode：用 FK 给末端真值。"""
    fk = TcpFK(urdf_path)
    z = np.load(str(path), allow_pickle=True)
    names = [str(x) for x in z["joint_names"]]
    q = np.asarray(z["joint_pos"], dtype=np.float64)

    rec = {s: z[f"{s}_pos"] for s in fk.ee_frames if f"{s}_pos" in z.files}
    recq = {s: z[f"{s}_quat"] for s in fk.ee_frames if f"{s}_quat" in z.files}
    base_src = "unset"
    if rec:
        fk.solve_base_from_rest(
            {s: p[0] for s, p in rec.items()},
            {s: v[0] for s, v in recq.items()} if recq else None,
        )
        base_src = "录制静止帧"

    poses = fk.tcp_poses(q, names, to_world=True)
    out: dict[str, Any] = {
        "episode": str(path),
        "frames": int(q.shape[0]),
        "dt_ms": (round(float(np.median(np.diff(z["t"]))) * 1000, 3)
                  if len(z["t"]) > 1 else None),
        "base_source": base_src,
        "base_t": None if fk.base_t is None else [round(float(v), 4) for v in fk.base_t],
        "urdf": str(fk.urdf_path),
    }
    for side, (p, _quat) in poses.items():
        out[side] = {
            "travel_mm": round(float(np.linalg.norm(p - p[0], axis=1).max() * 1000), 1),
            "z_min": round(float(np.nanmin(p[:, 2])), 4),
            "z_max": round(float(np.nanmax(p[:, 2])), 4),
        }
    if rec:
        out["validate"] = fk.validate_against(rec, recq, q, names)
    return out


def compute_cmd_poses(
    episode_path: str | Path,
    urdf_path: str | Path | None = None,
    to_world: bool = True,
) -> dict[str, tuple[np.ndarray, np.ndarray]]:
    """从 ``joint_pos_cmd`` 反算**末端指令位姿**。

    EE 回放（发 /left_target 给 OCS2）必须发指令目标，不能发实测位姿 ——
    否则运控把指令执行一次再产生误差 e，末端落在 目标−2e，手稳定偏下。

    指令位姿只存在于运控 MPC 内部，Isaac 侧只有 ``joint_pos_cmd``，所以用
    FK 反算。与运控的 ``eeFrame`` 同源（同一份 planning URDF），等价。

    Raises:
        ValueError: episode 没有 ``joint_pos_cmd``。
    """
    z = np.load(str(episode_path), allow_pickle=True)
    if "joint_pos_cmd" not in z.files:
        raise ValueError(
            f"{episode_path} 没有 joint_pos_cmd（下发目标关节角）。\n"
            "  EE 回放必须用指令位姿，不能用实测值。\n"
            "  重新采集：sdg.sh capture --what joint,ee,object"
        )
    fk = TcpFK(urdf_path)
    names = [str(x) for x in z["joint_names"]]
    q_cmd = np.asarray(z["joint_pos_cmd"], dtype=np.float64)

    if to_world:
        rest: dict[str, np.ndarray] = {}
        restq: dict[str, np.ndarray] = {}
        for side in fk.ee_frames:
            if f"{side}_pos" in z.files:
                rest[side] = z[f"{side}_pos"][0]
                if f"{side}_quat" in z.files:
                    restq[side] = z[f"{side}_quat"][0]
        if rest:
            fk.solve_base_from_rest(rest, restq or None)

    return fk.tcp_poses(q_cmd, names, to_world=to_world)


def compare_replay(
    replay_path: str | Path,
    source_episode: str | Path | None = None,
    urdf_path: str | Path | None = None,
) -> dict[str, Any]:
    """对比回放实际到达的末端 vs 录制目标/录制实测。

    两段结论：
    - ``sides``: 回放实际 vs 下发目标 —— 物理稳态误差（天生存在）
    - ``replay_vs_recording``: 回放实际 vs **采集实测** —— 这才是
      「回放有没有复现编排」的判据
    """
    fk = TcpFK(urdf_path)
    rp = np.load(str(replay_path), allow_pickle=True)
    target = np.asarray(rp["target"], dtype=np.float64)
    actual = np.asarray(rp["actual"], dtype=np.float64)
    frames = np.asarray(rp["frame"], dtype=np.int64)
    dof_names = [str(x) for x in rp["dof_names"]] if "dof_names" in rp.files else []
    if not dof_names:
        raise ValueError("replay 文件里没有 dof_names")

    base_src = "base 系（未提供 source episode）"
    ep = None
    if source_episode:
        ep = np.load(str(source_episode), allow_pickle=True)
        rest: dict[str, np.ndarray] = {}
        restq: dict[str, np.ndarray] = {}
        for side in fk.ee_frames:
            if f"{side}_pos" in ep.files:
                rest[side] = ep[f"{side}_pos"][0]
                if f"{side}_quat" in ep.files:
                    restq[side] = ep[f"{side}_quat"][0]
        if rest:
            fk.solve_base_from_rest(rest, restq or None)
            base_src = "录制静止帧"

    tgt_pose = fk.tcp_poses(target, dof_names, to_world=True)
    act_pose = fk.tcp_poses(actual, dof_names, to_world=True)

    joint_err = np.abs(actual - target)
    per_joint = {}
    for k, n in enumerate(dof_names):
        if k < joint_err.shape[1]:
            per_joint[n] = round(float(joint_err[:, k].max()) * 1000, 1)

    out: dict[str, Any] = {
        "replay": str(replay_path),
        "frames": int(len(frames)),
        "base_source": base_src,
        "urdf": str(fk.urdf_path),
        "joint_src": (str(rp["joint_src"].item())
                      if "joint_src" in rp.files else "?"),
        "sides": {},
    }
    for side in fk.ee_frames:
        d = np.linalg.norm(act_pose[side][0] - tgt_pose[side][0], axis=1) * 1000.0
        good = np.isfinite(d)
        if not good.any():
            continue
        dd = d[good]
        idx = int(np.argmax(dd))
        dv = act_pose[side][0][good][idx] - tgt_pose[side][0][good][idx]
        out["sides"][side] = {
            "mean_mm": round(float(dd.mean()), 1),
            "p95_mm": round(float(np.percentile(dd, 95)), 1),
            "max_mm": round(float(dd.max()), 1),
            "worst_frame": int(frames[good][idx]),
            "worst_delta_xyz_mm": [round(float(v) * 1000, 1) for v in dv],
            "travel_mm": round(float(np.linalg.norm(
                tgt_pose[side][0][-1] - tgt_pose[side][0][0]) * 1000), 1),
        }
    out["err_per_joint_mrad"] = dict(
        sorted(per_joint.items(), key=lambda kv: -kv[1])[:10]
    )

    # 关键判据：回放实测 vs 采集实测
    if ep is not None:
        ep_names = [str(x) for x in ep["joint_names"]]
        ep_q = np.asarray(ep["joint_pos"], dtype=np.float64)
        common = [n for n in dof_names if n in ep_names]
        if common:
            a_cols = [dof_names.index(n) for n in common]
            e_cols = [ep_names.index(n) for n in common]
            in_range = (frames >= 0) & (frames < ep_q.shape[0])
            if in_range.any():
                sub_a = actual[in_range][:, a_cols]
                sub_e = ep_q[frames[in_range]][:, e_cols]
                good = (np.isfinite(sub_a).all(axis=1)
                        & np.isfinite(sub_e).all(axis=1))
                if good.any():
                    ja = fk.tcp_poses(sub_a[good], common, to_world=True)
                    je = fk.tcp_poses(sub_e[good], common, to_world=True)
                    vs: dict[str, Any] = {}
                    for side in fk.ee_frames:
                        dd = np.linalg.norm(
                            ja[side][0] - je[side][0], axis=1) * 1000.0
                        okd = np.isfinite(dd)
                        if not okd.any():
                            continue
                        dd = dd[okd]
                        iw = int(np.argmax(dd))
                        dv = ja[side][0][okd][iw] - je[side][0][okd][iw]
                        vs[side] = {
                            "mean_mm": round(float(dd.mean()), 1),
                            "p95_mm": round(float(np.percentile(dd, 95)), 1),
                            "max_mm": round(float(dd.max()), 1),
                            "worst_frame": int(frames[in_range][good][okd][iw]),
                            "worst_delta_xyz_mm": [
                                round(float(v) * 1000, 1) for v in dv
                            ],
                        }
                    out["replay_vs_recording"] = vs
                    dq = np.abs(sub_a[good] - sub_e[good]) * 1000.0
                    pj = {n: round(float(dq[:, k].max()), 1)
                          for k, n in enumerate(common)}
                    out["replay_vs_recording_joint_mrad"] = dict(
                        sorted(pj.items(), key=lambda kv: -kv[1])[:8]
                    )
    return out


def compare_with_pinocchio(
    urdf_path: str | Path | None = None,
    joint_names: list[str] | None = None,
    n_samples: int = 5,
    seed: int = 0,
) -> dict[str, Any]:
    """自检：本模块的纯 numpy FK 与 pinocchio 对拍。

    pinocchio 只在这里作为**可选的验证工具**使用，正式路径不依赖它。
    项目里装了才跑；没装返回 ``{"skipped": ...}``。
    """
    try:
        import pinocchio as pin
    except ImportError:
        return {"skipped": "未安装 pinocchio（正式路径不需要它，仅用于对拍）"}

    fk = TcpFK(urdf_path)
    pm = pin.buildModelFromUrdf(str(fk.urdf_path))
    pdata = pm.createData()
    p_joint_order = [pm.names[i] for i in range(1, pm.njoints)]
    q_idx = [pm.joints[i].idx_q for i in range(1, pm.njoints)]
    fid = {s: pm.getFrameId(name, pin.FrameType.BODY) for s, name in fk.ee_frames.items()}

    rng = np.random.default_rng(seed)
    names = list(joint_names or fk.joint_order)
    cols = fk.map_from_joint_names(names)
    worst = 0.0
    for _ in range(n_samples):
        q = rng.uniform(-0.6, 0.6, size=len(names))
        got = fk.tcp_poses(q.reshape(1, -1), names, to_world=False)

        qp = pin.neutral(pm)
        for k, n in enumerate(p_joint_order):
            if n in names:
                qp[q_idx[k]] = float(q[names.index(n)])
        pin.forwardKinematics(pm, pdata, qp)
        pin.updateFramePlacements(pm, pdata)
        for side in fk.ee_frames:
            ref = np.asarray(pdata.oMf[fid[side]].translation)
            err = float(np.linalg.norm(got[side][0][0] - ref) * 1000)
            worst = max(worst, err)
    return {
        "ok": worst < 1.0,
        "worst_mm": round(worst, 4),
        "urdf": str(fk.urdf_path),
        "note": "纯 numpy FK 与 pinocchio 的位置差（应 < 1mm）",
    }


if __name__ == "__main__":  # pragma: no cover - 手工调试用
    import json
    import sys

    args = [a for a in sys.argv[1:] if not a.startswith("-")]
    if "--selfcheck" in sys.argv:
        print(json.dumps(compare_with_pinocchio(), indent=2, ensure_ascii=False))
    for p in args:
        print(json.dumps(analyze_episode(p), indent=2, ensure_ascii=False))
