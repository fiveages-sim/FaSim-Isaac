"""把 sdg 采集会话导出成 LeRobot 数据集格式（v3.0）。

为什么要这个
============
本项目采集的是**自定义 npz**（`frames/*.npz` + `episode.npz`），优点是写入极快
（每帧 1.3ms，走 savez 无压缩）、字段自由；缺点是不能直接被训练框架加载。

LeRobot 的 `LeRobotDataset` 是事实标准：`lerobot` 的 ACT/Diffusion 等策略可直接
读。所以补一个导出层，**采集侧保持快速，导出侧追求兼容**。

为什么不手写格式
----------------
LeRobot v3.0 的格式包含 parquet（帧数据+统计）、mp4（视频）、`meta/info.json`
（schema 版本、chunk 布局、统计）、`episodes.jsonl`、`tasks.jsonl` ……
字段语义与版本强耦合（v2.1→v3.0 就变过一次）。手写实现一旦格式升级就失效，
而且容易漏 stats（训练时会被用到）。

所以**直接调用官方 `LeRobotDataset`**：由它负责写盘与编码，我们只负责把
npz 里的字段喂进去。代价是导出需要 `lerobot` 的依赖（pyarrow/torch/av/…），
但这些在训练环境里本来就有。

依赖
----
需要能 `import lerobot`。本项目约定：
- 优先用 ``LE_ROBOT_PYTHON`` 环境变量指定的 python
- 否则找 ``~/miniconda3/envs/lerobot-ros2/bin/python``
- 都没有就报错并提示（不静默失败）

字段映射
========
LeRobot 的约定是 ``observation.*`` / ``action`` / ``task``：

    observation.state        ← joint_pos（实测关节角）        （observation）
    observation.ee_pos       ← <side>_pos（末端实测）          （observation）
    observation.ee_quat      ← <side>_quat
    observation.images.<cam> ← <cam>_rgb（uint8 HWC）          （observation）
    action                   ← joint_pos_cmd（**指令**，不是实测）

**为什么 action 用 joint_pos_cmd**：部署时策略只能输出命令，不能输出"结果"。
用实测当 action 会把运控的跟踪误差学进策略（实测末端比指令低 7~9mm），
这正是「抓取位置稳定偏下」的成因。见 episode.py 的说明。

额外字段（非 LeRobot 标准但保留，便于追溯）：
    observation.ee_pos_cmd / ee_quat_cmd   ← <side>_cmd_pos（末端指令位姿）

深度图与相机内参：LeRobot 的 image feature 只支持单通道 RGB。深度与 K
作为附加 `episode.meta` 写进 `meta/info.json` 的 custom 段（不参与训练，
但保留以便回溯）。若需要深度作为输入，后续可加 ``observation.images.<cam>_depth``
（需要改 feature dtype 与视频编码策略）。
"""

from __future__ import annotations

import json
import os
import shutil
import subprocess
import sys
from pathlib import Path
from typing import Any

import numpy as np

DEFAULT_ROBOT_TYPE = "unknown"


def _log(msg: str) -> None:
    print(f"[export] {msg}", flush=True)


# ---------------------------------------------------------------- 环境探测


def find_lerobot_python() -> Path | None:
    """找能 import lerobot 的 python。

    优先级：
      1. ``LE_ROBOT_PYTHON`` 环境变量
      2. 本仓库约定：``~/lerobot_ros2/.venv``（uv 迁移后的位置）
      3. 已知 conda 环境（过渡期保留，公司停用 conda 后可删）
      4. 当前解释器

    找到后返回路径；都不行返回 None（调用方报错，不静默降级）。
    """
    cands: list[str] = []
    env = os.environ.get("LE_ROBOT_PYTHON", "").strip()
    if env:
        cands.append(env)
    home = Path.home()
    cands += [
        # uv 环境（迁移后主用这个；.venv 用 --system-site-packages，能拿到系统 rclpy）
        str(home / "lerobot_ros2/.venv/bin/python"),
        # 过渡期：公司停用 conda 前的老环境
        str(home / "miniconda3/envs/lerobot-ros2/bin/python"),
        str(home / "miniconda3/envs/lerobot/bin/python"),
        sys.executable,
    ]
    seen: set[str] = set()
    for c in cands:
        if not c or c in seen:
            continue
        seen.add(c)
        p = Path(c)
        if not p.is_file():
            continue
        try:
            r = subprocess.run(
                [str(p), "-c", "import lerobot; print(lerobot.__file__)"],
                capture_output=True, text=True, timeout=30,
            )
            if r.returncode == 0:
                return p
        except Exception:  # noqa: BLE001
            continue
    return None


# ---------------------------------------------------------------- 数据读取


def _load_frames(session: Path) -> tuple[list[Path], dict]:
    frames = sorted((session / "frames").glob("frame_*.npz"))
    if not frames:
        raise FileNotFoundError(f"没有帧文件: {session / 'frames'}")
    meta: dict = {}
    sj = session / "session.json"
    if sj.is_file():
        try:
            meta = json.loads(sj.read_text(encoding="utf-8"))
        except Exception:  # noqa: BLE001
            pass
    return frames, meta


def _discover_cameras(frames: list[Path]) -> list[str]:
    with np.load(frames[0], allow_pickle=False) as z:
        return sorted(k[:-4] for k in z.files if k.endswith("_rgb"))


def _image_to_hwc_u8(arr: Any) -> np.ndarray | None:
    """Replicator 的 rgb 是 (H,W,4) uint8（RGBA）；LeRobot 要 (H,W,3)。"""
    a = np.asarray(arr)
    if a.ndim == 3 and a.shape[2] >= 3:
        a = a[..., :3]
    elif a.ndim == 2:
        a = np.stack([a] * 3, axis=-1)
    else:
        return None
    if a.dtype != np.uint8:
        a = np.clip(a, 0, 255).astype(np.uint8)
    return np.ascontiguousarray(a)


# ---------------------------------------------------------------- 主流程


#: 夹爪开合量的名字。**必须是 0/1 开关量**，不是关节位置。
GRIPPER_SWITCH_NAMES = ("left_gripper.pos", "right_gripper.pos")


def _gripper_switch_series(
    ep_npz: Path, n_frames: int
) -> list[tuple[float, float]]:
    """从 ``episode.npz`` 的 ``joint_pos_cmd`` 推出每帧的夹爪**开合开关量**。

    为什么是开关量而不是关节位置
    ----------------------------
    运控的 ``AdaptiveGripperController`` 是**开关量控制**（``target_command``，
    0=闭合 / 1=张开）。闭合时它不是回到"录制时的关节角"，而是回到**距初始值
    最近的限位**（``closed_position_``），靠压出夹持力夹紧物体
    （``adaptive_gripper_controller.cpp``::

        closed_position_ = (|upper - init| < |lower - init|) ? upper : lower;
        new_target = closed_position_ + percent * (open_position_ - closed_position_);

    ）。如果把采集到的夹爪关节位置当成指令发下去，闭合时会停在"刚好碰到管子"
    的位置，**夹持力不足** —— 这个坑踩过。所以这里只保留"开/合"这一位信息，
    幅度交给运控自己决定。

    为什么 sdg 内部就能拿到（不需要走话题）
    --------------------------------------
    sdg 采的是 Isaac 内部 ``get_dof_position_targets()``，而 ``target_command``
    模式下 ros2_control 只会往位置命令接口写**两个限位之一**，所以
    ``joint_pos_cmd`` 的夹爪列天然是离散的、逐帧带状态。实测 41 条 run 全部
    只有 3 个值 ``{-0.02, 0.0, +0.0245}``：

    ==========  ==========================================
    cmd         实际后果（run_01 物理结果反推）
    ==========  ==========================================
    ``-0.0200`` 夹住：右手持管抬起，tube_z 0.743 → 0.920
    ``+0.0245`` 张开：管子落回 tube_z 0.743
    ``0.0000``  上电闲置值 —— **运控还没下发过指令**
    ==========  ==========================================

    ### idle 值必须「保持上一帧」，不能判成闭合

    ``0.0`` 恰好等于 URDF 初始值。物理上两指此时是并拢的（确实像闭合），
    但**它不是"意图闭合"**，而是"运控还没发第一个命令"。而真实工况是：
    **进 OCS2 后夹爪会张开**，推理开始时应保持张开。

    早期实现把 ``0.0`` 判成闭合，导致训练数据教模型"开局左爪闭合"
    （实测 41 条 run 的 ``left_gripper_cmd[0]`` 全为 0，而左爪首次张开要到
    第 147~151 帧）。推理时表现为"开局左爪就闭合"，与场景不符。

    所以：**``|v - init| < eps`` 的帧沿用上一帧的值**（没有上一帧时用 1.0=张开，
    与"进 OCS2 后张开"的工况一致）。

    其余帧按"离哪个限位更近"判定 —— 复刻控制器规则，而不是"高于/低于中点"
    （后者在初始值靠近上限的机器人上会反掉）。

    Args:
        ep_npz: ``episode.npz``（pack 产物，含 ``joint_pos_cmd`` + ``joint_names``）。
        n_frames: 期望帧数（与 frames 对齐）。

    Returns:
        ``[(left_sw, right_sw), ...]``，1=张开、0=闭合。
    """
    import numpy as np

    with np.load(ep_npz, allow_pickle=False) as z:
        keys = set(z.files)
        jn = [str(n) for n in z["joint_names"]] if "joint_names" in keys else []
        q = z["joint_pos_cmd"] if "joint_pos_cmd" in keys else None
    if q is None or not jn:
        # 没有指令列 → 不敢猜，全部按张开处理（至少不会误夹）
        return [(1.0, 1.0)] * n_frames
    q = np.asarray(q, dtype=np.float64)

    # 每条手臂解析出 (closed_val, open_val, init_val)；判据 = 离哪个更近。
    bounds: list[tuple[float, float, float] | None] = []
    for side in ("left", "right"):
        col = None
        for cand in (f"{side}_gripper_joint", f"{side}_hand_finger_joint"):
            if cand in jn:
                col = jn.index(cand)
                break
        if col is None:
            bounds.append(None)
            continue
        seq = q[:, col]
        finite = seq[np.isfinite(seq)]
        if not len(finite):
            bounds.append(None)
            continue
        lo, hi = float(finite.min()), float(finite.max())
        if hi - lo < 1e-6:
            # 全程同一个值 —— 该 episode 没动过夹爪，按张开处理
            bounds.append(None)
            continue
        # 初始值：第一条有效帧。它通常等于 URDF 初始位（0.0），也正因为它
        # 是"运控尚未下发"的 idle 值，所以判定时要单独排除（见下面的 idle 处理）。
        init = float(finite[0])
        # 复刻 AdaptiveGripperController：距初始值更近的限位 = 闭合位
        if abs(hi - init) < abs(lo - init):
            bounds.append((hi, lo, init))     # (closed, open, init)
        else:
            bounds.append((lo, hi, init))

    # idle 判定的容差：与初始值相差小于"到两个限位距离"的 5% 就算 idle。
    # 用相对量而不是绝对值，换机器人/换限位范围时不用改。
    def _is_idle(v: float, closed_val: float, open_val: float, init: float) -> bool:
        span = max(abs(open_val - closed_val), 1e-9)
        return abs(v - init) < 0.05 * span

    out_sw: list[tuple[float, float]] = []
    # 起始默认 **张开**：真实工况是"进 OCS2 后夹爪张开"，推理开始时就是张开。
    # 若首帧是 idle（没下过指令），保持张开是唯一自洽的选择。
    prev = [1.0, 1.0]
    for i in range(n_frames):
        row: list[float] = []
        for k, bnd in enumerate(bounds):
            col = None
            side = ("left", "right")[k]
            for cand in (f"{side}_gripper_joint", f"{side}_hand_finger_joint"):
                if cand in jn:
                    col = jn.index(cand)
                    break
            if bnd is None or col is None:
                row.append(prev[k])
                continue
            v = float(q[i, col]) if i < len(q) else float("nan")
            if not np.isfinite(v):
                row.append(prev[k])
                continue
            closed_val, open_val, init = bnd
            if _is_idle(v, closed_val, open_val, init):
                # idle（运控还没下发）→ **保持上一帧**。
                # 首帧时 prev 是 1.0（张开），与"进 OCS2 后夹爪张开"一致。
                row.append(prev[k])
                continue
            # 离张开位更近 → 1；否则 → 0
            row.append(1.0 if abs(v - open_val) < abs(v - closed_val) else 0.0)
        while len(row) < 2:
            row.append(1.0)
        prev = row[:2]
        out_sw.append((row[0], row[1]))
    return out_sw


def _load_ee_actions(
    ep_npz: Path, n_frames: int, with_gripper: bool = False
) -> list[list[float]]:
    """从 episode.npz 读末端**绝对**位姿作为 action（可选带夹爪开关量）。

    字段顺序必须与 features["action"]["names"] 一致：
        left_ee.pos.{x,y,z}, left_ee.quat.{x,y,z,w},
        right_ee.pos.{x,y,z}, right_ee.quat.{x,y,z,w}
        [, left_gripper.pos, right_gripper.pos]   ← with_gripper=True

    用 ``*_cmd_pos/_cmd_quat``（**FK 反算的指令位姿**），不是 ``*_pos/_quat``
    （实测值）：实测与指令有 6.7mm 稳态误差，拿实测当目标发运控会让误差叠加
    两次。这与「回放必须先发指令」是同一条原则。

    夹爪位（when ``with_gripper``）取自 :func:`_gripper_switch_series` ——
    **开关量 0/1**，不是关节位置。详见那个函数的说明。

    帧数对齐：episode.npz 与 frames/*.npz 由同一个 pack 生成，应逐帧对应。
    若长度不符则按最短对齐并告警（宁可少几帧也不要错位）。
    """
    import numpy as np

    out: list[list[float]] = []
    with np.load(ep_npz, allow_pickle=False) as z:
        keys = set(z.files)
        need = [f"{s}_cmd_{k}" for s in ("left", "right") for k in ("pos", "quat")]
        missing = [k for k in need if k not in keys]
        if missing:
            raise ValueError(
                f"episode.npz 缺字段 {missing} —— 它可能是旧版 pack 产物。"
                f"重跑 sdg.sh pack 即可（会用 FK 反算末端指令位姿）"
            )
        lp = np.asarray(z["left_cmd_pos"], dtype=np.float32).reshape(-1, 3)
        lq = np.asarray(z["left_cmd_quat"], dtype=np.float32).reshape(-1, 4)
        rp = np.asarray(z["right_cmd_pos"], dtype=np.float32).reshape(-1, 3)
        rq = np.asarray(z["right_cmd_quat"], dtype=np.float32).reshape(-1, 4)
    n = min(len(lp), len(lq), len(rp), len(rq))
    if n != n_frames:
        _log(f"WARN {ep_npz.parent.name}: episode.npz 有 {n} 帧，"
             f"frames 有 {n_frames} 帧 —— 按 {min(n, n_frames)} 对齐")
        n = min(n, n_frames)

    grip = _gripper_switch_series(ep_npz, n) if with_gripper else None

    for i in range(n):
        # 转成 Python float —— numpy.float32 不能直接 JSON 序列化。
        # 下游 worker 通过 json 传参，这里是唯一出口，统一转一次。
        #
        # ⚠️ **四元数必须把 wxyz 转成 xyzw**。
        # ``episode.npz`` 里的 ``*_cmd_quat`` 是 ``fk.py`` 的 ``_rot_to_wxyz``
        # 产物，顺序是 **w,x,y,z**；而我们在 features 里声明的 action 名字是
        # ``left_ee.quat.x/.y/.z/.w``（对齐 geometry_msgs/Pose，那是 **x,y,z,w**）。
        #
        # 直接按原名搬会把整条链错位一位，实际表示变成"绕 Z 转 180° + 怪旋转"，
        # 实测表现为左右臂镜像、夹爪朝正后方扭转 —— 而位置部分正常，
        # 所以只有姿态诡异、很迷惑。
        row = [
            float(lp[i, 0]), float(lp[i, 1]), float(lp[i, 2]),
            # wxyz -> xyzw
            float(lq[i, 1]), float(lq[i, 2]), float(lq[i, 3]), float(lq[i, 0]),
            float(rp[i, 0]), float(rp[i, 1]), float(rp[i, 2]),
            float(rq[i, 1]), float(rq[i, 2]), float(rq[i, 3]), float(rq[i, 0]),
        ]
        if grip is not None:
            # 0/1 开关量：0=闭合（运控回限位压出夹持力）、1=张开
            row.append(float(grip[i][0]))
            row.append(float(grip[i][1]))
        out.append(row)
    return out


def export_session(
    session_dir: str | Path,
    out_dir: str | Path,
    *,
    repo_id: str | None = None,
    fps: float | None = None,
    robot_type: str | None = None,
    task: str | None = None,
    use_videos: bool = True,
    overwrite: bool = False,
    lerobot_python: str | Path | None = None,
    action_space: str = "absolute",
) -> dict:
    """把一个采集会话导出成 LeRobot 数据集。

    Args:
        session_dir: 采集会话目录（含 frames/ 与 session.json）。
        out_dir: 输出目录（LeRobot 数据集根）。
        repo_id: LeRobot 的 repo_id（默认用会话名）。
        fps: 数据帧率；默认由 ``physics_dt × every`` 推。
        robot_type: 写进 info.json 的机器人类型。
        task: 任务文本（LeRobot 的 `task` 字段）；默认用会话目录名。
        use_videos: True 时图像编码成 mp4（体积小，需要 av）。
            纯调试可设 False，但 LeRobot v3 要求有 video 或 image feature。
        overwrite: 输出目录已存在时是否清空。
        lerobot_python: 指定 python；默认自动找。
        action_space: ``action`` 字段的语义（默认 ``absolute``）：
            **absolute** —— ``joint_pos_cmd``（绝对关节指令）。部署直接输出
                目标角，无累积误差。**推荐用于自训练**。
            **delta** —— ``joint_pos[t+1] - joint_pos[t]``（相邻**实测**位移）。
                满足 ``state[t] + action[t] == state[t+1]``（误差 0），与同事
                data-collector-platform 语义一致，便于对拍/混训。
                ⚠ 不是 ``cmd - 实测``：那是控制器修正量，实测比位移大 7.4 倍
                （修正里绝大部分被稳态误差吃掉）。
                代价：最后一帧无 t+1，action 填 0；部署需积分，长序列会漂。
            **ee_delta** —— 末端位姿相邻增量（pos3+dquat4 × 左右 = 14 维），
                与同事 pose 模式完全对齐。同样用实测值而非 cmd。
            **ee_absolute** —— 末端**绝对**位姿（pos3+quat4 × 左右 = 14 维），
                对齐 ``inference.py`` 直接发运控。位姿取 ``*_cmd_pos/_cmd_quat``
                （FK 反算的**指令**位姿），不是实测。
            **ee_absolute_gripper** —— 上面 14 维 + 2 维夹爪**开关量**
                （``left_gripper.pos`` / ``right_gripper.pos``，0=闭合 1=张开
                = 运控 ``target_command`` 的语义）。夹爪只保留开/合这一位，
                幅度交给运控的 ``AdaptiveGripperController`` 自己回限位压出
                夹持力 —— **不要**把采集到的夹爪关节位置当指令，那样夹不紧。

    Returns:
        ``{"ok", "out", "frames", "episodes", "features", "python"}``

    说明：``session_dir`` 可以是单个会话路径，也可以是**会话路径列表**
    （多 episode 导出 —— 每个会话导出成数据集里的一个 episode）。
    后者用于「一个任务批次下多次采集 → 一个训练集」的场景。
    """
    import tempfile

    raw_sessions = ([session_dir] if isinstance(session_dir, (str, Path))
                    else list(session_dir))
    sessions = [Path(s).expanduser().resolve() for s in raw_sessions]
    if not sessions:
        raise ValueError("没有给出任何会话目录")
    for s in sessions:
        if not s.is_dir():
            raise FileNotFoundError(s)
    session = sessions[0]   # 主会话：用于默认 repo_id / robot_type / task
    out = Path(out_dir).expanduser().resolve()
    if out.exists():
        if not overwrite:
            raise FileExistsError(
                f"输出目录已存在: {out}\n  加 --overwrite 覆盖，或换一个 --out"
            )
        shutil.rmtree(out)

    # 逐会话收集帧 —— 每个会话是一个独立 episode
    episodes_frames: list[list[Path]] = []
    metas: list[dict] = []
    used_sessions: list[Path] = []
    for s in sessions:
        fs, m = _load_frames(s)
        if not fs:
            _log(f"WARN 跳过无帧会话: {s}")
            continue
        episodes_frames.append(fs)
        metas.append(m)
        used_sessions.append(s)
    if not episodes_frames:
        raise ValueError("所有会话都没有帧")

    # ``ee_absolute`` 需要 FK 反算的末端指令位姿（``left_cmd_pos`` 等），
    # 那些字段**只存在于 episode.npz**（pack 时由纯 numpy FK 算出），
    # 原始 frames/*.npz 里没有。所以这里先从 episode.npz 读出来，按
    # 每个 episode 一组传给 worker。
    #
    # 为什么不在 worker 里读：worker 只拿到帧文件路径清单，不认识会话目录
    # 结构；而且 episode 与 frames 的帧数/顺序需要用同一个来源保证一致。
    asp_pre = str(action_space or "absolute").lower()
    want_grip = "gripper" in asp_pre
    ee_actions: list[list[list[float]]] | None = None
    if asp_pre in ("ee_absolute", "ee_absolute_gripper"):
        ee_actions = []
        for s, fs in zip(used_sessions, episodes_frames):
            ep_npz = s / "episode.npz"
            if not ep_npz.is_file():
                raise ValueError(
                    f"{s.name} 缺 episode.npz —— ee_absolute 需要 FK 反算的末端指令位姿。\n"
                    f"  先跑: sdg.sh pack {s}  （或对任务目录跑一次批量 pack）"
                )
            acts = _load_ee_actions(ep_npz, len(fs), with_gripper=want_grip)
            ee_actions.append(acts)
        bad = sum(1 for a in ee_actions for r in a if not any(r))
        if bad:
            _log(f"WARN {bad} 帧的 ee action 全零（episode.npz 里缺 cmd 字段？）")
        else:
            _log(f"ee action 已从 episode.npz 读取（{len(ee_actions)} 个 episode"
                 + ("，含夹爪开关量" if want_grip else "") + "）")
        if want_grip and ee_actions:
            g = ee_actions[0]
            nl = sum(1 for r in g if len(r) > 15 and r[14] < 0.5)
            nr = sum(1 for r in g if len(r) > 15 and r[15] < 0.5)
            _log(f"  夹爪闭合帧（值=0）: left {nl}/{len(g)}  right {nr}/{len(g)}")
    frames = [f for ep in episodes_frames for f in ep]   # 扁平（用于探测 feature）
    meta = metas[0]
    n = len(frames)
    cams = _discover_cameras(frames)
    if len(episodes_frames) > 1:
        _log(f"多会话导出：{len(episodes_frames)} 个 episode，"
             f"共 {n} 帧 → {out}")

    # fps：优先 session.json 的 physics_dt×every。
    # physics_dt 是**设定值**（场景 physxScene:timeStepsPerSecond → 仿真时间口径），
    # 不是墙钟实际步长 —— 用后者会让 fps 偏低（本机 177 vs 200Hz，差 13%）。
    if fps is None:
        pdt = float(meta.get("physics_dt", 0) or 0)
        if pdt <= 0 and (meta.get("physics_hz_setting") or 0) > 0:
            pdt = 1.0 / float(meta["physics_hz_setting"])
        every = int(meta.get("every", 1) or 1)
        fps = 1.0 / (pdt * every) if pdt > 0 else 20.0
    fps_i = max(1, int(round(float(fps))))

    repo = repo_id or session.name
    rtype = robot_type or str(meta.get("robot_key") or DEFAULT_ROBOT_TYPE)
    task_text = task or f"{meta.get('robot_key', '')}/{meta.get('task_key', '')}".strip("/") \
        or session.name

    _log(f"会话: {session}")
    _log(f"  {n} 帧  相机: {cams}  fps={fps_i}  robot_type={rtype}")
    _log(f"输出: {out}")

    py = Path(lerobot_python) if lerobot_python else find_lerobot_python()
    if py is None:
        raise RuntimeError(
            "找不到能 import lerobot 的 python。\n"
            "  设 LE_ROBOT_PYTHON=/path/to/python，或激活装了 lerobot 的环境。\n"
            "  本项目默认用 ~/miniconda3/envs/lerobot-ros2/bin/python。"
        )
    _log(f"使用 python: {py}")

    # 探测一帧确定 feature 形状
    with np.load(frames[0], allow_pickle=False) as z:
        jn = [str(x) for x in z["joint_names"]] if "joint_names" in z.files else []
        q = np.asarray(z["joint_pos"]).reshape(-1)
        nj = int(q.size)
        ee_dim = 0
        for side in ("left", "right"):
            if f"{side}_pos" in z.files:
                ee_dim += 7  # pos3 + quat4
        img_hw = None
        for c in cams:
            a = _image_to_hwc_u8(z[f"{c}_rgb"])
            if a is not None:
                img_hw = a.shape[:2]
                break

    asp = str(action_space or "absolute").lower()
    if asp not in ("absolute", "delta", "ee_delta", "ee_absolute",
                   "ee_absolute_gripper"):
        raise ValueError(
            f"action_space 必须是 absolute / delta / ee_delta / ee_absolute"
            f" / ee_absolute_gripper，收到 {action_space!r}"
        )

    features: dict[str, Any] = {
        "observation.state": {
            "dtype": "float32", "shape": [nj], "names": jn or None,
        },
    }
    # action 的形状随语义变：
    #   absolute / delta → 关节维（nj）
    #   ee_delta         → 左右各 pos3+dquat4 = 14（**增量**）
    #   ee_absolute      → 左右各 pos3+quat4 = 14（**绝对**，对齐 inference.py）
    #   ee_absolute_gripper → 上面 14 维 + 左右夹爪开关量 2 维 = 16
    if asp == "ee_delta":
        features["action"] = {
            "dtype": "float32",
            "shape": [14],
            "names": [f"{s}_{k}" for s in ("left", "right")
                      for k in ("dpos_x", "dpos_y", "dpos_z",
                                "dquat_x", "dquat_y", "dquat_z", "dquat_w")],
        }
    elif asp in ("ee_absolute", "ee_absolute_gripper"):
        # 名字必须与 lerobot_robot_ros2 的 send_action 对得上：
        #   LEFT_EE_PREFIX = "left_ee" → 它读 left_ee.pos.x / left_ee.quat.x ...
        #   LEFT_GRIPPER_KEY = "left_gripper.pos"
        # 所以这里用 left_ee.pos.x 这种**带下划线**的写法，而不是 left_pos_x。
        names = [f"{s}_ee.{kind}.{axis}"
                 for s in ("left", "right")
                 for kind, axes in (("pos", "xyz"), ("quat", "xyzw"))
                 for axis in axes]
        if asp == "ee_absolute_gripper":
            names += list(GRIPPER_SWITCH_NAMES)
        features["action"] = {
            "dtype": "float32",
            "shape": [len(names)],
            "names": names,
        }
    else:
        a_names = None
        if jn:
            if asp == "delta":
                a_names = [f"d{n}" for n in jn]
            else:
                a_names = jn
        features["action"] = {
            "dtype": "float32", "shape": [nj], "names": a_names,
        }
    # 末端实测位姿（pos3+quat4 × 左右）—— observation
    if ee_dim:
        features["observation.ee"] = {"dtype": "float32", "shape": [ee_dim]}
    if img_hw is not None:
        for c in cams:
            features[f"observation.images.{c}"] = {
                "dtype": "video",
                "shape": [img_hw[0], img_hw[1], 3],
                "names": ["height", "width", "channels"],
            }
    # 末端指令位姿 —— 追溯 action 的等效末端目标（不参与训练，仅保留）
    cmd_dim = 0
    with np.load(frames[0], allow_pickle=False) as z:
        for side in ("left", "right"):
            if f"{side}_cmd_pos" in z.files:
                cmd_dim += 7
    if cmd_dim:
        features["observation.ee_cmd"] = {"dtype": "float32", "shape": [cmd_dim]}

    # 交互物体位姿（obj/<label>_pos + _quat，每个物体 7 维，按名字排序保证顺序稳定）
    obj_labels: list[str] = []
    with np.load(frames[0], allow_pickle=False) as z:
        obj_labels = sorted(
            k[len("obj/"):-len("_pos")] for k in z.files
            if k.startswith("obj/") and k.endswith("_pos")
            and f"obj/{k[len('obj/'):-len('_pos')]}_quat" in z.files
        )
    for lab in obj_labels:
        features[f"observation.objects.{lab}"] = {
            "dtype": "float32", "shape": [7],
        }

    # 交给子进程跑（因为 lerobot 依赖在另一个 python 里）
    req = {
        "session": str(session),
        "out": str(out),
        "repo_id": repo,
        "fps": fps_i,
        "robot_type": rtype,
        "task": task_text,
        "frames": [str(f) for f in frames],
        # 多 episode：每个子列表导出成数据集里的一个 episode。
        # worker 优先用这个；没有时回落到上面的扁平 frames（单 episode）。
        "episodes": [[str(f) for f in ep] for ep in episodes_frames],
        # ee_absolute 专用：每个 episode 一组 14 维绝对位姿动作
        "ee_actions": ee_actions,
        "episode_names": [s.name for s in used_sessions],
        "cams": cams,
        "features": features,
        "obj_labels": obj_labels,
        "use_videos": bool(use_videos),
        "action_space": asp,
        "meta": {
            "seed": {
                "source_session": str(session),
                "robot_key": meta.get("robot_key"),
                "task_key": meta.get("task_key"),
                "physics_dt": meta.get("physics_dt"),
                "every": meta.get("every"),
                "code_stamp": meta.get("code_stamp"),
                "cameras": cams,
            },
        },
    }
    req_path = Path(tempfile.mkstemp(suffix=".json")[1])
    req_path.write_text(json.dumps(req, ensure_ascii=False), encoding="utf-8")
    try:
        worker = Path(__file__).with_name("_lerobot_export_worker.py")
        r = subprocess.run([str(py), str(worker), str(req_path)])
        if r.returncode != 0:
            raise RuntimeError(f"导出失败（worker 退出码 {r.returncode}）")
    finally:
        req_path.unlink(missing_ok=True)

    return {
        "ok": True, "out": str(out), "frames": n,
        "episodes": len(episodes_frames),
        "features": sorted(features), "python": str(py),
    }
