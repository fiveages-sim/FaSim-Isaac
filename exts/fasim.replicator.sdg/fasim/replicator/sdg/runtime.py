"""SDG 运行时路径与默认值 —— 唯一的"硬编码"出口。

设计原则
========
**其余所有模块都不许出现绝对路径**，一律从这里取。这样：

- 仓库 clone 到任何位置都能跑（扩展根目录运行时探测）
- 用户数据目录可通过环境变量覆盖（多用户 / 多磁盘）
- CLI、扩展、渲染器共享同一套默认值，不会各说各话

环境变量（都可选）
==================
``FASIM_SDG_DATA``      采集输出根目录。默认 ``~/fasim_data``
``FASIM_SDG_RUNTIME``   运行时状态目录（socket / status）。默认 ``<data>/.runtime``
``FASIM_SDG_HOME``      扩展根目录。默认自动探测（本文件向上 4 层）

为什么默认输出是 ``~/fasim_data`` 而不是 ``~/fasim_live``
========================================================
``fasim_live`` 是早期调试产物，混了 control.txt 和半成品数据；
``fasim_data`` 语义明确（数采数据），且和 ``~/fa_data``（层1 的 npz）
区分：前者是层2.5 的相机数据，后者是层1 的轨迹。
"""

from __future__ import annotations

import os
from pathlib import Path

#: 包内的配置根（configs/<robot>/...）
PACKAGE_ROOT = Path(__file__).resolve().parent

#: 扩展根目录：<ext>/fasim/replicator/sdg/__file__ → 向上 4 层
_DEFAULT_HOME = PACKAGE_ROOT.parents[2]


def ext_home() -> Path:
    """扩展根目录（含 config/extension.toml 与 scripts/）。"""
    raw = os.environ.get("FASIM_SDG_HOME", "").strip()
    return Path(raw).expanduser().resolve() if raw else _DEFAULT_HOME


def data_root() -> Path:
    """采集数据根目录。"""
    raw = os.environ.get("FASIM_SDG_DATA", "").strip()
    return Path(raw).expanduser().resolve() if raw else Path.home() / "fasim_data"


def runtime_dir() -> Path:
    """运行时状态目录（Unix socket、status 快照）。"""
    raw = os.environ.get("FASIM_SDG_RUNTIME", "").strip()
    if raw:
        return Path(raw).expanduser().resolve()
    return data_root() / ".runtime"


def socket_path() -> Path:
    """扩展与 CLI 之间的 Unix domain socket。

    socket 而不是文件轮询的原因：
      - CLI 能拿到**同步响应**（已启动 / 帧数 / 错误），不用读日志猜测
      - 没有"文件残留导致误触发"的问题
      - 延迟从"最多一帧"降到即时
    sockaddr_un 路径上限 108 字节（Linux），这里会校验。
    """
    return runtime_dir() / "capture.sock"


def status_path() -> Path:
    """最近一次采集的状态快照（JSON），供 ``sdg.sh status`` 读取。"""
    return runtime_dir() / "status.json"


def ensure_runtime_dir() -> Path:
    d = runtime_dir()
    d.mkdir(parents=True, exist_ok=True)
    return d


def default_session_dir(robot: str, task: str, name: str = "") -> Path:
    """<data>/<robot>/<task>/<name or 时间戳>/"""
    import datetime

    stamp = name.strip() or datetime.datetime.now().strftime("%Y%m%d-%H%M%S")
    return data_root() / robot / task / stamp


def list_sessions(
    robot: str | None = None,
    task: str | None = None,
    *,
    with_episode_only: bool = False,
    limit: int = 50,
) -> list[dict]:
    """扫描数据根目录下的采集会话，供交互菜单/CLI **列表选择**。

    为什么要这个：手输路径既慢又容易打错（``run_01`` 打成 ``run_1``），而且
    用户通常记不住完整路径。这里直接列目录，带帧数/时间/是否有 episode。

    目录约定（``default_session_dir``）::

        <data_root>/<robot>/<task>/<session_name>/
            session.json    采集元信息（帧数/相机/robot_key…）
            frames/         逐帧 npz
            episode.npz     pack 的产物（可能还没生成）

    非标准位置（如 ``<data_root>/cam03``）也会被收进来 —— 它们没有
    robot/task 层级，靠 session.json 里的 robot_key/task_key 归属。

    Args:
        robot/task: 过滤条件；``None`` 表示不限。
        with_episode_only: 只要已经 pack 出 ``episode.npz`` 的会话。
        limit: 最多返回多少条（按 mtime 倒序，最新的在前）。

    Returns:
        每项::

            {"path": Path, "name": str, "robot": str, "task": str,
             "frames": int, "duration": float, "has_episode": bool,
             "mtime": float, "cameras": [..]}
    """
    import json

    root = data_root()
    if not root.is_dir():
        return []

    out: list[dict] = []
    for sj in root.rglob("session.json"):
        sess = sj.parent
        # 跳过 .runtime 之类的隐藏目录
        if any(p.startswith(".") for p in sess.relative_to(root).parts):
            continue
        try:
            meta = json.loads(sj.read_text(encoding="utf-8"))
        except Exception:  # noqa: BLE001
            meta = {}
        r = str(meta.get("robot_key") or "")
        t = str(meta.get("task_key") or "")
        # 没有 robot_key 的老会话：从路径猜 <data>/<robot>/<task>/<name>
        if not r or not t:
            rel = sess.relative_to(root).parts
            if len(rel) >= 3:
                r, t = (r or rel[0]), (t or rel[1])
        if robot and r != robot:
            continue
        if task and t != task:
            continue
        ep = sess / "episode.npz"
        if with_episode_only and not ep.is_file():
            continue
        out.append({
            "path": sess,
            "name": sess.name,
            "robot": r,
            "task": t,
            "frames": int(meta.get("frames_saved", 0) or 0),
            "duration": float(meta.get("duration_sec", 0) or 0),
            "cameras": [c.get("name") for c in (meta.get("cameras") or [])],
            "has_episode": ep.is_file(),
            # capturing = 采集未正常结束（Isaac 崩溃 / 忘了 stop）
            "incomplete": str(meta.get("status") or "") == "capturing",
            "mtime": sj.stat().st_mtime,
        })

    out.sort(key=lambda d: -d["mtime"])
    return out[:limit]
