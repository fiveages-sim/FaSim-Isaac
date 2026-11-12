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

    Returns:
        ``{"ok", "out", "frames", "features", "python"}``
    """
    import tempfile

    session = Path(session_dir).expanduser().resolve()
    out = Path(out_dir).expanduser().resolve()
    if not session.is_dir():
        raise FileNotFoundError(session)
    if out.exists():
        if not overwrite:
            raise FileExistsError(
                f"输出目录已存在: {out}\n  加 --overwrite 覆盖，或换一个 --out"
            )
        shutil.rmtree(out)

    frames, meta = _load_frames(session)
    n = len(frames)
    cams = _discover_cameras(frames)

    # fps：优先 session.json 的 physics_dt×every
    if fps is None:
        pdt = float(meta.get("physics_dt", 0) or 0)
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

    features: dict[str, Any] = {
        "observation.state": {
            "dtype": "float32", "shape": [nj], "names": jn or None,
        },
        "action": {
            "dtype": "float32", "shape": [nj], "names": jn or None,
        },
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

    # 交给子进程跑（因为 lerobot 依赖在另一个 python 里）
    req = {
        "session": str(session),
        "out": str(out),
        "repo_id": repo,
        "fps": fps_i,
        "robot_type": rtype,
        "task": task_text,
        "frames": [str(f) for f in frames],
        "cams": cams,
        "features": features,
        "use_videos": bool(use_videos),
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
        "features": sorted(features), "python": str(py),
    }
