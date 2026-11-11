"""用关节角做正解算 tcp 位姿 —— 替代 Isaac 里读不到的末端位姿。

为什么需要这个
==============
本项目的末端位姿（``left_tcp`` / ``right_tcp``）在 Isaac 里**读不到动态值**。

实测证据（``diag01``，765 帧）：``XformPrim.get_world_poses()`` 返回的
``left_pos`` / ``right_pos`` 与首帧偏离 **0.0mm**，也就是恒定不变。进一步
核对发现它的值恰好等于 **q≈0 静止姿态** 下的 tcp 世界位姿 —— 读到的是
authoring stage 的静态值。

根因在 newton kit（``isaacsim.exp.full.newton.kit``）::

    [settings.exts."omni.physx"]
    updateToUsd                     = false
    fabricUpdateTransformations     = false

即使运行时把求解器切回 PhysX，**PhysX 也不会把变换写回 USD/Fabric**，
所以 ``XformPrim`` 无论用 ``usd`` / ``usdrt`` / ``fabric`` 哪个后端都拿不到
物理更新后的值。给 ``use_backend("fabric")`` 打补丁是救不了的。

（物体位姿能读，是因为 ``RigidPrim`` 走 tensor 后端直读 PhysX，不经过
USD/Fabric 同步管道。）

为什么 FK 是更好的方案
======================
- **不依赖任何同步管道**：纯数学，给定关节角必得位姿。
- **与运控同源**：用 OCS2 实际加载的 planning URDF（``mobile_base_tcp.info``
  的 ``eeFrame = left_tcp`` / ``eeFrame1 = right_tcp``），所以 FK 出来的
  就是运控 IK/MPC 用的那个 frame，定义上不会差。
- **可离线复算**：采完的数据随时能重算，不用重启 Isaac。

坐标系约定
==========
FK 输出在 **base 系**（URDF 根 = ``base_footprint``）。episode 里记的末端
位姿在 **world 系**，两者差一个 base 位姿。

``solve_base_from_rest()`` 用静止姿态那一帧反解 ``world_from_base``：机器人
上电时关节角全是 ~0，而 tcp 相对 base 是固定几何量，所以只要有一帧
（base 位姿已知的）静止姿态就能定出变换。

注意：底盘在编排里是 LOCK 的（``BASE_LOCK``），所以 base 位姿在所有帧
保持不变，解一次就够。

依赖
====
``pinocchio``。Isaac 的 python 里通常没有，所以本模块**默认不在 Isaac
进程内使用** —— 采集侧只存关节角，分析侧离线算。需要时显式传
``urdf_path``；拿不到 pinocchio 就返回 ``None`` 而不是抛异常，避免拖垮
采集回调。
"""

from __future__ import annotations

import json
from pathlib import Path
from typing import Any

import numpy as np

#: 规划 URDF 候选位置（按优先级）。OCS2 编译缓存里的那份是**运行时真正用的**，
#: 所以排第一；`xacro` 现生成的那份作为兜底。
PLANNING_URDF_CANDIDATES = (
    "/tmp/ocs2_ros2/unitree_g1d/planning_urdf/{hash}.urdf",
    "/tmp/g1d_urdf/g1d.urdf",
)


def _log(msg: str) -> None:
    print(f"[fk] {msg}", flush=True)


def find_planning_urdf() -> Path | None:
    """找 OCS2 实际加载的 planning URDF。

    OCS2 把（URDF + .info 的 hash）编到 ``/tmp/ocs2_ros2/<robot>/planning_urdf/``
    下。可能有多个 hash（改过 URDF），取 mtime 最新的那个 —— 与运控当前
    使用的是同一份。
    """
    root = Path("/tmp/ocs2_ros2")
    if root.is_dir():
        cands = sorted(
            root.glob("*/planning_urdf/*.urdf"),
            key=lambda p: p.stat().st_mtime,
            reverse=True,
        )
        if cands:
            return cands[0]
    for pat in PLANNING_URDF_CANDIDATES[1:]:
        p = Path(pat)
        if p.is_file():
            return p
    return None


class TcpFK:
    """从关节角算左右 tcp 位姿（base 系），并可换算到 world 系。

    Args:
        urdf_path: planning URDF；``None`` → 自动找。
        ee_frames: ``{"left": "left_tcp", "right": "right_tcp"}``。

    Attributes:
        joint_order: URDF 里的驱动关节顺序（**不要用 articulation 的 DOF
            顺序**，两者不同 —— G1D 的 articulation 是 32 DOF 且顺序乱序，
            规划 URDF 是 18 个运控关节）。
    """

    def __init__(
        self,
        urdf_path: str | Path | None = None,
        ee_frames: dict[str, str] | None = None,
    ) -> None:
        import pinocchio as pin

        self._pin = pin
        p = Path(urdf_path) if urdf_path else find_planning_urdf()
        if p is None or not Path(p).is_file():
            raise FileNotFoundError(
                "找不到 planning URDF。先跑一次运控生成 /tmp/ocs2_ros2/.../planning_urdf/，"
                "或用 xacro 生成后显式传 urdf_path"
            )
        self.urdf_path = Path(p)
        self.model = pin.buildModelFromUrdf(str(self.urdf_path))
        self.data = self.model.createData()

        # 关节名 → (q 索引)。prismatic 也是 1 个 q，所以 nq == 关节数。
        self.joint_order: list[str] = [
            self.model.names[i] for i in range(1, self.model.njoints)
        ]
        self._q_index = [self.model.joints[i].idx_q for i in range(1, self.model.njoints)]

        self.ee_frames = dict(ee_frames or {"left": "left_tcp", "right": "right_tcp"})
        self._frame_id = {}
        for side, name in self.ee_frames.items():
            fid = self.model.getFrameId(name)
            if fid >= self.model.nframes:
                raise ValueError(f"URDF 里没有 frame {name!r}（{self.urdf_path}）")
            self._frame_id[side] = fid

        #: ``world_from_base`` 的平移 / 旋转，由 ``solve_base_from_rest`` 填。
        self.base_t: np.ndarray | None = None
        self.base_R: np.ndarray | None = None

    # ------------------------------------------------------------ 关节映射

    def map_from_joint_names(self, joint_names: list[str]) -> np.ndarray:
        """给定数据里的关节名顺序 → URDF q 顺序的取列索引。

        Raises:
            ValueError: 缺关节（回放会算错位，必须显式失败而不是静默填零）。
        """
        names = [str(n) for n in joint_names]
        missing = [n for n in self.joint_order if n not in names]
        if missing:
            raise ValueError(f"数据里缺少 URDF 关节 {missing}")
        return np.array([names.index(n) for n in self.joint_order], dtype=np.int64)

    # ------------------------------------------------------------ FK

    def _fk_one(self, q18: np.ndarray) -> dict[str, tuple[np.ndarray, np.ndarray]]:
        pin = self._pin
        q = pin.neutral(self.model)
        for k, qi in enumerate(self._q_index):
            q[qi] = float(q18[k])
        pin.forwardKinematics(self.model, self.data, q)
        pin.updateFramePlacements(self.model, self.data)
        out = {}
        for side, fid in self._frame_id.items():
            m = self.data.oMf[fid]
            out[side] = (np.asarray(m.translation).copy(), _rot_to_wxyz(m.rotation))
        return out

    def tcp_poses(
        self,
        joint_pos: np.ndarray,
        joint_names: list[str],
        to_world: bool = True,
    ) -> dict[str, tuple[np.ndarray, np.ndarray]]:
        """整段轨迹的 tcp 位姿。

        Args:
            joint_pos: ``(N, J)`` 关节角。
            joint_names: 对应 ``J`` 列的名字。
            to_world: 换算到 world 系（需先调用 ``solve_base_from_rest``）。

        Returns:
            ``{side: (pos (N,3), quat_wxyz (N,4))}``。
        """
        q = np.asarray(joint_pos, dtype=np.float64)
        cols = self.map_from_joint_names(joint_names)
        n = q.shape[0]
        res: dict[str, tuple[np.ndarray, np.ndarray]] = {}
        for side in self.ee_frames:
            res[side] = (np.zeros((n, 3)), np.zeros((n, 4)))

        for i in range(n):
            row = q[i][cols]
            if not np.all(np.isfinite(row)):
                for side in self.ee_frames:
                    res[side][0][i] = np.nan
                    res[side][1][i] = np.nan
                continue
            one = self._fk_one(row)
            for side, (p, quat) in one.items():
                if to_world and self.base_R is not None:
                    p = self.base_R @ p + self.base_t
                    quat = _quat_mul(_rot_to_wxyz(self.base_R), quat)
                res[side][0][i] = p
                res[side][1][i] = quat
        return res

    def tcp_pose_at(
        self,
        q18_by_name: dict[str, float],
        to_world: bool = True,
    ) -> dict[str, tuple[np.ndarray, np.ndarray]]:
        """单帧（按名字给关节角）—— 回放里逐帧诊断用。"""
        row = np.array([float(q18_by_name[n]) for n in self.joint_order])
        one = self._fk_one(row)
        if to_world and self.base_R is not None:
            return {
                s: (self.base_R @ p + self.base_t,
                    _quat_mul(_rot_to_wxyz(self.base_R), quat))
                for s, (p, quat) in one.items()
            }
        return one

    # ------------------------------------------------------------ base 反解

    def solve_base_from_rest(
        self,
        rest_pos: dict[str, np.ndarray],
        rest_quat: dict[str, np.ndarray] | None = None,
    ) -> tuple[np.ndarray, np.ndarray]:
        """用静止姿态的 tcp 世界位姿反解 ``world_from_base``。

        机器人上电时关节角 ≈ 0，tcp 相对 base 是固定几何量，所以一帧静止
        姿态就够定出 base 位姿。左右手各解一次再平均，抵消单侧噪声。

        Args:
            rest_pos: ``{"left": (3,), "right": (3,)}`` —— world 系。
            rest_quat: 可选 ``{"left": (4,), ...}``（wxyz）。给了就用姿态
                联合求解（更稳）；不给只用平移。

        Returns:
            ``(base_t (3,), base_R (3,3))``，同时写回 ``self``。
        """
        q0 = np.zeros(len(self.joint_order))
        rest_fk = self._fk_one(q0)

        # --- 平移：t_world = R_wb @ p_base + t_wb ---
        # 先只用姿态求 R（若给了 quat），再最小二乘求 t。
        R_wb = None
        if rest_quat:
            Rs = []
            for side, quat in rest_quat.items():
                if side not in rest_fk:
                    continue
                qw = np.asarray(quat, dtype=np.float64).reshape(-1)[:4]
                R_rec = _wxyz_to_rot(qw)
                R_fk = _wxyz_to_rot(rest_fk[side][1])
                Rs.append(R_rec @ R_fk.T)
            if Rs:
                R_wb = _average_rotation(Rs)
        if R_wb is None:
            # 没有姿态信息：假设只有 yaw（G1D 在平地上，roll/pitch≈0）
            # 用左右手连线方向定 yaw。
            if {"left", "right"} <= rest_pos.keys():
                d_world = np.asarray(rest_pos["right"])[:2] - np.asarray(rest_pos["left"])[:2]
                d_base = rest_fk["right"][0][:2] - rest_fk["left"][0][:2]
                yaw = float(np.arctan2(d_world[1], d_world[0]) - np.arctan2(d_base[1], d_base[0]))
                c, s = np.cos(yaw), np.sin(yaw)
                R_wb = np.array([[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]])
            else:
                R_wb = np.eye(3)

        # t 用所有可用的手做最小二乘
        diffs = []
        for side, p_world in rest_pos.items():
            if side in rest_fk:
                diffs.append(np.asarray(p_world, dtype=np.float64).reshape(-1)[:3]
                             - R_wb @ rest_fk[side][0])
        if not diffs:
            raise ValueError("rest_pos 里没有和 ee_frames 对应的手")
        t_wb = np.mean(np.stack(diffs), axis=0)

        self.base_R = R_wb
        self.base_t = t_wb
        return t_wb, R_wb

    # ------------------------------------------------------------ 自检

    def validate_against(
        self,
        rec_pos: dict[str, np.ndarray],
        rec_quat: dict[str, np.ndarray],
        joint_pos: np.ndarray,
        joint_names: list[str],
        max_mm: float = 5.0,
    ) -> dict[str, Any]:
        """核对录制的末端位姿是否真的是动态值。

        这是**采集链路的体检**：如果录制值与 FK 差了几米、或录制值自身
        完全不变化，说明位姿采集坏了（本项目就踩过这个坑）。

        Returns:
            ``{"ok": bool, "per_side": {...}, "reason": str}``。
        """
        fk = self.tcp_poses(joint_pos, joint_names, to_world=True)
        per: dict[str, Any] = {}
        ok = True
        reasons = []
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
            rec_travel = float(np.nanmax(np.linalg.norm(
                rec[np.isfinite(rec).all(axis=1)] - rec[np.isfinite(rec).all(axis=1)][0],
                axis=1)) * 1000.0) if np.isfinite(rec).all(axis=1).any() else 0.0
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
                reasons.append(f"{side}: 录制值恒定为静态值（FK 动了 {fk_travel:.0f}mm）")
        return {"ok": ok, "per_side": per, "reason": "; ".join(reasons) or "正常"}


# ---------------------------------------------------------------- 旋转工具


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
    """旋转矩阵 → 四元数 (w,x,y,z)。"""
    R = np.asarray(R, dtype=np.float64)
    tr = R[0, 0] + R[1, 1] + R[2, 2]
    if tr > 0:
        s = np.sqrt(tr + 1.0) * 2
        w = 0.25 * s
        x = (R[2, 1] - R[1, 2]) / s
        y = (R[0, 2] - R[2, 0]) / s
        z = (R[1, 0] - R[0, 1]) / s
    elif R[0, 0] > R[1, 1] and R[0, 0] > R[2, 2]:
        s = np.sqrt(1.0 + R[0, 0] - R[1, 1] - R[2, 2]) * 2
        w = (R[2, 1] - R[1, 2]) / s
        x = 0.25 * s
        y = (R[0, 1] + R[1, 0]) / s
        z = (R[0, 2] + R[2, 0]) / s
    elif R[1, 1] > R[2, 2]:
        s = np.sqrt(1.0 + R[1, 1] - R[0, 0] - R[2, 2]) * 2
        w = (R[0, 2] - R[2, 0]) / s
        x = (R[0, 1] + R[1, 0]) / s
        y = 0.25 * s
        z = (R[1, 2] + R[2, 1]) / s
    else:
        s = np.sqrt(1.0 + R[2, 2] - R[0, 0] - R[1, 1]) * 2
        w = (R[1, 0] - R[0, 1]) / s
        x = (R[0, 2] + R[2, 0]) / s
        y = (R[1, 2] + R[2, 1]) / s
        z = 0.25 * s
    q = np.array([w, x, y, z], dtype=np.float64)
    return q / np.linalg.norm(q)


def _quat_mul(a: np.ndarray, b: np.ndarray) -> np.ndarray:
    """四元数乘法（wxyz），a∘b。"""
    aw, ax, ay, az = (float(v) for v in np.asarray(a).reshape(-1)[:4])
    bw, bx, by, bz = (float(v) for v in np.asarray(b).reshape(-1)[:4])
    return np.array([
        aw * bw - ax * bx - ay * by - az * bz,
        aw * bx + ax * bw + ay * bz - az * by,
        aw * by - ax * bz + ay * bw + az * bx,
        aw * bz + ax * by - ay * bx + az * bw,
    ])


def _average_rotation(Rs: list[np.ndarray]) -> np.ndarray:
    """多个旋转矩阵的平均（SVD 投影，保证结果仍是正交矩阵）。"""
    M = np.mean(np.stack(Rs), axis=0)
    U, _S, Vt = np.linalg.svd(M)
    R = U @ Vt
    if np.linalg.det(R) < 0:  # 反射修正
        U[:, -1] *= -1
        R = U @ Vt
    return R


# ---------------------------------------------------------------- 离线入口


def compute_cmd_poses(
    episode_path: str | Path,
    urdf_path: str | Path | None = None,
    to_world: bool = True,
) -> dict[str, tuple[np.ndarray, np.ndarray]]:
    """从 ``joint_pos_cmd`` 反算**末端指令位姿**。

    为什么需要
    ----------
    EE 回放（``drive="ee"``，发 /left_target 给 OCS2）必须发**指令目标**，
    不能发实测位姿 —— 否则和关节回放踩同一个坑：运控把指令执行一次产生
    误差 e，末端落在 目标 − e；若目标本身已经是含 e 的实测值，就变成
    目标 − 2e，手稳定偏下。``ee_traj.py`` 里也是这个逻辑：优先
    ``*_cmd_pos``，没有才回退实测。

    但采集时拿不到末端指令位姿：
    - 它是运控内部 MPC 解出来的，Isaac 侧只有关节目标 ``joint_pos_cmd``
    - ROS 的 ``/left_current_target`` 需要额外订阅（违反"数采走 Isaac 内部"）

    所以用 FK 从 ``joint_pos_cmd`` 算 —— 和运控自己的 ``eeFrame`` 定义同源
    （同一个 planning URDF），是等价的。

    Args:
        episode_path: 含 ``joint_pos_cmd`` 的 episode。
        urdf_path: planning URDF；``None`` → 自动找。
        to_world: 换算到 world（需要能解 base，用实测末端定）。

    Returns:
        ``{"left": (pos (N,3), quat_wxyz (N,4)), "right": ...}``。

    Raises:
        ValueError: episode 没有 ``joint_pos_cmd``。
    """
    z = np.load(str(episode_path), allow_pickle=True)
    if "joint_pos_cmd" not in z.files:
        raise ValueError(
            f"{episode_path} 没有 joint_pos_cmd（下发目标关节角）。\n"
            "  EE 回放必须用指令位姿，不能用实测值 —— 否则末端会多偏一次。\n"
            "  重新采集：sdg.sh capture --what joint,ee,object"
        )
    fk = TcpFK(urdf_path)
    names = [str(x) for x in z["joint_names"]]
    q_cmd = np.asarray(z["joint_pos_cmd"], dtype=np.float64)

    if to_world:
        # base 位姿要用**实测**末端 + 实测关节角解（那才是真实几何）
        rest = {}
        restq = {}
        q_act = np.asarray(z["joint_pos"], dtype=np.float64)
        for side in fk.ee_frames:
            if f"{side}_pos" in z.files:
                rest[side] = z[f"{side}_pos"][0]
                if f"{side}_quat" in z.files:
                    restq[side] = z[f"{side}_quat"][0]
        if rest:
            # 用实测关节角算 FK base —— 实测末端对应实测关节角
            fk.solve_base_from_rest(
                {s: p for s, p in rest.items()},
                {s: v for s, v in restq.items()} if restq else None,
            )
        del q_act

    return fk.tcp_poses(q_cmd, names, to_world=to_world)


def analyze_episode(path: str | Path, urdf_path: str | Path | None = None) -> dict:
    """离线体检一个 episode：用 FK 给末端真值，报运动学量。

    纯 numpy/pinocchio，不开 Isaac。
    """
    fk = TcpFK(urdf_path)
    z = np.load(str(path), allow_pickle=True)
    names = [str(x) for x in z["joint_names"]]
    q = np.asarray(z["joint_pos"], dtype=np.float64)

    # base 反解：优先用录制末端（若它是好的），否则假设 base 在原点。
    rec = {s: z[f"{s}_pos"] for s in fk.ee_frames if f"{s}_pos" in z.files}
    recq = {s: z[f"{s}_quat"] for s in fk.ee_frames if f"{s}_quat" in z.files}
    base_src = "unset"
    if rec:
        fk.solve_base_from_rest({s: p[0] for s, p in rec.items()},
                                {s: v[0] for s, v in recq.items()} if recq else None)
        base_src = "录制静止帧"

    poses = fk.tcp_poses(q, names, to_world=True)
    out: dict[str, Any] = {
        "episode": str(path),
        "frames": int(q.shape[0]),
        "dt_ms": round(float(np.median(np.diff(z["t"]))) * 1000, 3) if len(z["t"]) > 1 else None,
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


def compare_replay(
    replay_path: str | Path,
    source_episode: str | Path | None = None,
    urdf_path: str | Path | None = None,
) -> dict[str, Any]:
    """对比「回放实际到达的关节角」与「录制目标」的**末端**偏差。

    这是"手偏 2-3cm"的直接量化 —— 之前用 Xform 读位姿得到 0.0mm 是假的，
    这里用 FK 对 target 与 actual 各算一次 tcp 位姿再作差。

    Args:
        replay_path: ``live_replay`` 导出的 ``*.replay_fN.npz``
            （含 ``target`` / ``actual`` / ``frame``）。
        source_episode: 原始 episode，用于解 ``world_from_base``（拿静止帧
            tcp 位姿）。不给则只能用 base 系（差值仍然有效，只是绝对位置不同）。

    Returns:
        逐帧末端偏差统计 + 最大偏差帧 + 逐关节误差归因。
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
    if source_episode:
        ep = np.load(str(source_episode), allow_pickle=True)
        rest = {}
        restq = {}
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
        "sides": {},
    }
    for side in fk.ee_frames:
        d = np.linalg.norm(act_pose[side][0] - tgt_pose[side][0], axis=1) * 1000.0
        good = np.isfinite(d)
        if not good.any():
            continue
        d = d[good]
        idx = int(np.argmax(d))
        dv = act_pose[side][0][good][idx] - tgt_pose[side][0][good][idx]
        out["sides"][side] = {
            "mean_mm": round(float(d.mean()), 1),
            "p95_mm": round(float(np.percentile(d, 95)), 1),
            "max_mm": round(float(d.max()), 1),
            "worst_frame": int(frames[good][idx]),
            "worst_delta_xyz_mm": [round(float(v) * 1000, 1) for v in dv],
            "travel_mm": round(float(np.linalg.norm(
                tgt_pose[side][0][-1] - tgt_pose[side][0][0]) * 1000), 1),
        }
    out["err_per_joint_mrad"] = dict(
        sorted(per_joint.items(), key=lambda kv: -kv[1])[:10]
    )

    # ---- 关键判据：回放实测 vs 采集实测 ----
    #
    # 上面 ``sides`` 量的是 drive 稳态误差（物理本来就该有，不是缺陷）。
    # 真正要验证的是：**回放时手动到的地方，和编排时手到的地方一样吗**。
    # 所以拿回放的 actual 去比原 episode 的 joint_pos（也是实测）。
    if source_episode:
        ep = np.load(str(source_episode), allow_pickle=True)
        ep_names = [str(x) for x in ep["joint_names"]]
        ep_q = np.asarray(ep["joint_pos"], dtype=np.float64)
        common = [n for n in dof_names if n in ep_names]
        if common:
            a_cols = [dof_names.index(n) for n in common]
            e_cols = [ep_names.index(n) for n in common]
            rows = frames[(frames >= 0) & (frames < ep_q.shape[0])]
            sub_a = actual[(frames >= 0) & (frames < ep_q.shape[0])][:, a_cols]
            sub_e = ep_q[rows][:, e_cols]
            good = np.isfinite(sub_a).all(axis=1) & np.isfinite(sub_e).all(axis=1)
            if good.any():
                ja = fk.tcp_poses(sub_a[good], common, to_world=True)
                je = fk.tcp_poses(sub_e[good], common, to_world=True)
                vs: dict[str, Any] = {}
                for side in fk.ee_frames:
                    dd = np.linalg.norm(ja[side][0] - je[side][0], axis=1) * 1000.0
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
                        "worst_frame": int(rows[good][okd][iw]),
                        "worst_delta_xyz_mm": [round(float(v) * 1000, 1) for v in dv],
                    }
                out["replay_vs_recording"] = vs
                # 关节角层面的差（更贴近"数据对不对"）
                dq = np.abs(sub_a[good] - sub_e[good]) * 1000.0
                pj = {n: round(float(dq[:, k].max()), 1)
                      for k, n in enumerate(common)}
                out["replay_vs_recording_joint_mrad"] = dict(
                    sorted(pj.items(), key=lambda kv: -kv[1])[:8]
                )
    return out



if __name__ == "__main__":  # pragma: no cover - 手工调试用
    import sys

    for arg in sys.argv[1:]:
        print(json.dumps(analyze_episode(arg), indent=2, ensure_ascii=False))
