"""统一的 episode 数据格式（npz）—— 层1 与 层2 之间唯一的契约。

设计原则
========
1. **以关节名为准，不以索引为准。**
   运控发的是 ``/joint_states`` 里按名字的 18 个关节；Isaac 的
   ``ArticulationView`` 有 32 个 DOF（含 4 驱动轮 + 8 脚轮关节），顺序也不同。
   只存索引会在两者之间错位，回放出来的姿态就是错的。所以 npz 里同时存
   ``joint_names``，回放时按名字映射。

2. **一帧一个时间点，帧号即时间轴。**
   回放是逐帧步进的（``app.update()`` 一次 = 一帧），所以对齐用
   **帧索引**而不是时间戳。``t`` 只用于日志/可视化，不参与对齐。
   这样彻底避免"墙钟抖动 / sim clock 暂停"导致的时间戳漂移。

3. **末端与关节角是同一次采样的两份视图。**
   ``ee_*`` 用于对接以末端为目标的运控与训练；
   ``joint_*`` 用于精确重建姿态（渲染相机）。
   两者在录制时同一时刻读，回放时只信 ``joint_*``。

字段
====
标量/向量（形状均为 ``(N, ...)``，N = 帧数）::

    t                 (N,)      相对秒数，仅日志用
    joint_pos         (N, J)    实测关节角（rad / m），按 joint_names 排列
    joint_vel         (N, J)    实测关节速度
    left_pos          (N, 3)    左末端实测位置（world）
    left_quat         (N, 4)    左末端实测姿态 xyzw
    right_pos         (N, 3)
    right_quat        (N, 4)
    left_cmd_pos      (N, 3)    左末端指令目标（可选）
    left_cmd_quat     (N, 4)
    right_cmd_pos     (N, 3)
    right_cmd_quat    (N, 4)
    left_grip         (N,)      夹爪指令 1=开 0=关 -1=未知
    right_grip        (N,)

元信息::

    joint_names       (J,)      字符串数组，joint_pos 的列序
    frame_id                  末端位姿的坐标系（如 "world"）
    meta                       JSON 字符串（robot/task/来源话题等）
    schema_version            格式版本
"""

from __future__ import annotations

import json
import os
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any

import numpy as np

SCHEMA_VERSION = 2

#: 回放时不参与的 DOF 名字片段（底盘驱动/脚轮）——底盘姿态不在轨迹里，
#: 驱动它们会让整个底座被拖动。
NON_MANIPULATION_TOKENS = ("wheel", "caster")

#: 视为夹爪的 DOF 名字片段。夹爪在层2 按「开关量」回放而不是位置回放，
#: 原因见 ``renderer._build_gripper_switches``。
GRIPPER_NAME_TOKENS = ("gripper", "hand_", "finger")


@dataclass
class Episode:
    """一段可回放的轨迹。见模块 docstring 的字段表。"""

    t: np.ndarray
    joint_pos: np.ndarray
    joint_names: list[str]
    left_pos: np.ndarray
    left_quat: np.ndarray
    right_pos: np.ndarray
    right_quat: np.ndarray
    joint_vel: np.ndarray | None = None
    left_cmd_pos: np.ndarray | None = None
    left_cmd_quat: np.ndarray | None = None
    right_cmd_pos: np.ndarray | None = None
    right_cmd_quat: np.ndarray | None = None
    left_grip: np.ndarray | None = None
    right_grip: np.ndarray | None = None
    #: 交互物体位姿：label -> (pos(N,3), quat(N,4))。独立刚体，回放时需显式设定。
    objects: dict[str, tuple[np.ndarray, np.ndarray]] | None = None
    frame_id: str = "world"
    meta: dict[str, Any] = field(default_factory=dict)

    def __len__(self) -> int:
        return int(self.t.shape[0])

    @property
    def num_joints(self) -> int:
        return int(self.joint_pos.shape[1])

    # ---------------------------------------------------------------- 校验

    def validate(self) -> None:
        n = len(self)
        if n == 0:
            raise ValueError("episode 为空")
        if self.joint_pos.shape[0] != n:
            raise ValueError(
                f"joint_pos 帧数 {self.joint_pos.shape[0]} != t 帧数 {n}"
            )
        if len(self.joint_names) != self.num_joints:
            raise ValueError(
                f"joint_names 数量 {len(self.joint_names)} != joint_pos 列数 {self.num_joints}"
            )
        for name in ("left_pos", "right_pos"):
            arr = getattr(self, name)
            if arr.shape != (n, 3):
                raise ValueError(f"{name} 形状应为 {(n, 3)}，实际 {arr.shape}")
        for name in ("left_quat", "right_quat"):
            arr = getattr(self, name)
            if arr.shape != (n, 4):
                raise ValueError(f"{name} 形状应为 {(n, 4)}，实际 {arr.shape}")
        if not np.all(np.isfinite(self.joint_pos)):
            bad = int((~np.isfinite(self.joint_pos)).sum())
            total = int(self.joint_pos.size)
            # 采集开头运控未就绪时，前几帧关节角是 NaN（物体沉降段）。
            # 只要坏值占比不高，就允许通过。
            #
            # 整列 NaN 不视为错误：mimic 关节（如 Dex1 的 *_gripper_joint1）
            # 不出现在 /joint_states 里，采集端按固定列序只能填 NaN。
            # 回放时这些列会被逐帧跳过，由 USD 的 MimicJointAPI 自行跟随
            # 主关节，不影响姿态复现。
            entire_nan_cols = np.where(
                ~np.isfinite(self.joint_pos).any(axis=0)
            )[0]
            if entire_nan_cols.size:
                names = [self.joint_names[c] for c in entire_nan_cols]
                print(
                    f"[warn] {len(names)} 个关节整列为 NaN，回放时跳过: {names}",
                    flush=True,
                )
            ratio = bad / max(1, total)
            if ratio > 0.5:
                raise ValueError(
                    f"joint_pos 有 {bad}/{total} ({ratio*100:.1f}%) 个 NaN，"
                    "太多，检查采集时序"
                )
            print(
                f"[warn] joint_pos 含 {bad}/{total} ({ratio*100:.2f}%) 个 NaN"
                "（通常是采集开头运控未就绪的帧）"
            )

    # ---------------------------------------------------------------- 存取

    def save(self, path: str | os.PathLike) -> Path:
        self.validate()
        p = Path(path).expanduser()
        p.parent.mkdir(parents=True, exist_ok=True)
        payload: dict[str, Any] = {
            "schema_version": np.array(SCHEMA_VERSION),
            "t": self.t,
            "joint_pos": self.joint_pos,
            "joint_names": np.array(self.joint_names),
            "left_pos": self.left_pos,
            "left_quat": self.left_quat,
            "right_pos": self.right_pos,
            "right_quat": self.right_quat,
            "frame_id": np.array(self.frame_id),
            "meta": np.array(json.dumps(self.meta, ensure_ascii=False)),
        }
        for name in (
            "joint_vel",
            "left_cmd_pos", "left_cmd_quat",
            "right_cmd_pos", "right_cmd_quat",
            "left_grip", "right_grip",
        ):
            value = getattr(self, name)
            if value is not None:
                payload[name] = value
        # 交互物体位姿：obj/<label>_pos / obj/<label>_quat
        if self.objects:
            for label, (pos, quat) in self.objects.items():
                payload[f"obj/{label}_pos"] = pos
                payload[f"obj/{label}_quat"] = quat
        np.savez_compressed(p, **payload)
        return p

    @classmethod
    def load(cls, path: str | os.PathLike) -> "Episode":
        p = Path(path).expanduser()
        if not p.exists():
            raise FileNotFoundError(p)
        with np.load(p, allow_pickle=False) as z:
            files = set(z.files)

            # 旧格式（scripts/dof_replay.py 的 dof_episode_*.npz）只有 dof_positions，
            # 没有关节名 → 无法安全映射到 articulation，必须拒绝而不是猜。
            if "t" not in files:
                if "dof_positions" in files:
                    raise ValueError(
                        f"{p} 是旧的 dof_replay 格式（仅 dof_positions，无关节名）。\n"
                        "  它无法安全映射到 Isaac 的 DOF 顺序，已停止加载。\n"
                        "  请用新的采集方式重新录制：\n"
                        "    ee_traj.sh record -o <out>.npz\n"
                        "  新格式包含 joint_names，是层2 回放的唯一定量依据。"
                    )
                raise ValueError(
                    f"{p} 缺少字段 't'，不是 episode 格式。实际字段: {sorted(files)}"
                )

            meta_raw = z["meta"].item() if "meta" in files else "{}"
            if isinstance(meta_raw, bytes):
                meta_raw = meta_raw.decode("utf-8")

            def opt(name: str):
                return z[name] if name in files else None

            # 交互物体：收集 obj/<label>_pos 的 label
            objects: dict[str, tuple[np.ndarray, np.ndarray]] = {}
            for key in files:
                if key.startswith("obj/") and key.endswith("_pos"):
                    label = key[len("obj/"):-len("_pos")]
                    quat_key = f"obj/{label}_quat"
                    if quat_key in files:
                        objects[label] = (z[key], z[quat_key])

            return cls(
                t=z["t"],
                joint_pos=z["joint_pos"],
                joint_names=[str(s) for s in z["joint_names"]],
                left_pos=z["left_pos"],
                left_quat=z["left_quat"],
                right_pos=z["right_pos"],
                right_quat=z["right_quat"],
                joint_vel=opt("joint_vel"),
                left_cmd_pos=opt("left_cmd_pos"),
                left_cmd_quat=opt("left_cmd_quat"),
                right_cmd_pos=opt("right_cmd_pos"),
                right_cmd_quat=opt("right_cmd_quat"),
                left_grip=opt("left_grip"),
                right_grip=opt("right_grip"),
                objects=objects or None,
                frame_id=z["frame_id"].item() if "frame_id" in files else "world",
                meta=json.loads(str(meta_raw)),
            )

    # ---------------------------------------------------------------- 摘要

    def summary(self) -> str:
        n = len(self)
        dur = float(self.t[-1] - self.t[0]) if n > 1 else 0.0
        dt = np.diff(self.t) if n > 1 else np.array([0.0])
        lines = [
            f"帧数        : {n}",
            f"时长        : {dur:.2f} s",
            f"帧间隔      : median {float(np.median(dt))*1e3:.1f} ms",
            f"关节数      : {self.num_joints}",
            f"末端坐标系  : {self.frame_id}",
            f"左末端行程  : {_path_len(self.left_pos)*100:.1f} cm",
            f"右末端行程  : {_path_len(self.right_pos)*100:.1f} cm",
        ]
        if self.left_grip is not None:
            lines.append(f"左夹爪      : {_grip_desc(self.left_grip)}")
        if self.right_grip is not None:
            lines.append(f"右夹爪      : {_grip_desc(self.right_grip)}")
        return "\n".join(lines)


def _path_len(pos: np.ndarray) -> float:
    """末端行程（米）。跳过 NaN 帧——采集开头运控未就绪时位姿是 NaN。"""
    if pos.shape[0] < 2:
        return 0.0
    ok = np.isfinite(pos).all(axis=1)
    if ok.sum() < 2:
        return 0.0
    p = pos[ok]
    d = np.linalg.norm(np.diff(p, axis=0), axis=1)
    # 跨 NaN 段产生的巨大跳变会被误算成行程，用中位数过滤掉
    if d.size > 3:
        med = float(np.median(d))
        d = d[d <= med * 20 + 0.5]
    return float(d.sum())


def _grip_desc(arr: np.ndarray) -> str:
    known = arr[arr >= 0]
    if known.size == 0:
        return "未采集"
    changes = int((np.abs(np.diff(known)) > 0.5).sum())
    n_open = int((known > 0.5).sum())
    return f"开 {n_open} / 关 {known.size - n_open} 帧，切换 {changes} 次"


# ---------------------------------------------------------------------------
# 关节名映射（层2 回放的核心）
# ---------------------------------------------------------------------------

def build_dof_mapping(
    dof_names: list[str], episode_joint_names: list[str]
) -> tuple[np.ndarray, np.ndarray, list[str]]:
    """把 episode 的关节列映射到 Isaac articulation 的 DOF 索引。

    Returns:
        ``(target_idx, source_idx, missing)``
        - ``target_idx``: 要写入的 DOF 索引
        - ``source_idx``: 对应的 episode 列索引
        - ``missing``: episode 里有、但 articulation 里找不到的关节名

    未出现在映射里的 DOF（底盘轮/脚轮等）保持当前值不动。
    """
    dof_index = {name: i for i, name in enumerate(dof_names)}
    target_idx: list[int] = []
    source_idx: list[int] = []
    missing: list[str] = []
    for col, name in enumerate(episode_joint_names):
        idx = dof_index.get(name)
        if idx is None:
            missing.append(name)
            continue
        target_idx.append(idx)
        source_idx.append(col)
    return (
        np.asarray(target_idx, dtype=np.int64),
        np.asarray(source_idx, dtype=np.int64),
        missing,
    )


def skipped_dof_names(dof_names: list[str]) -> list[str]:
    """回放时不会驱动的 DOF（底盘相关）。"""
    return [n for n in dof_names if any(t in n.lower() for t in NON_MANIPULATION_TOKENS)]
