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
