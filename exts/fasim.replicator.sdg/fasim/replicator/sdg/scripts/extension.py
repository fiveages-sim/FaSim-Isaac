"""扩展启动钩子：随 Isaac Sim 自动加载，通过**控制文件**触发采集。

为什么用控制文件而不是 Script Editor
====================================
Script Editor 中文乱码、每次要粘贴，不适合反复调试。这里在扩展
``on_startup`` 里起一个轻量轮询（用 ``omni.kit.app`` 的 update 事件，
不额外开线程、不阻塞主循环），监听一个控制文件：

    控制文件默认: ~/fasim_live/control.txt

写什么就做什么（读完即清空，避免重复触发）::

    start            用默认参数开始采集
    start every=20   带参数开始（也支持 camera=, out=, width=, height=）
    stop             结束采集并写 session.json
    status           打印当前状态

等价于在终端::

    echo start > ~/fasim_live/control.txt
    echo stop  > ~/fasim_live/control.txt

这样跑编排时不用切窗口、不用碰 Script Editor。

为什么不用后台线程轮询文件
==========================
``omni.kit.app`` 的 update 事件已经在主循环里，每帧调一次回调、内部只是
``os.path.exists`` + ``read``，开销可忽略。开线程反而要处理 GIL 和
跨线程调用 Replicator 的安全问题。
"""

from __future__ import annotations

import omni.ext
import omni.kit.app

#: 控制文件路径。扩展加载后会在日志里打印它，可直接 echo 写入。
CONTROL_FILE = "~/fasim_live/control.txt"

#: 采集器实例（延迟导入，避免扩展加载时就去 import replicator）
_CAPTURE = None
_APP = None
_UPDATE_SUB = None
_LAST = {"exists": False, "mtime": 0.0}


def _log(msg: str) -> None:
    print(f"[live_capture] {msg}", flush=True)


def _ensure_capture():
    """延迟导入采集模块（此时 kit 已就绪，replicator 可安全 import）。"""
    global _CAPTURE
    if _CAPTURE is not None:
        return _CAPTURE
    import sys
    from pathlib import Path

    # extension.py 在 <ext>/fasim/replicator/sdg/scripts/extension.py
    # parents[0]=scripts, [1]=sdg, [2]=replicator, [3]=fasim, [4]=<ext>
    ext_root = Path(__file__).resolve().parents[4]
    scripts = ext_root / "scripts"
    for p in (str(ext_root), str(scripts)):
        if p not in sys.path:
            sys.path.insert(0, p)

    import live_capture as lc  # noqa: PLC0415

    _CAPTURE = lc
    return lc


def _parse_cmd(text: str) -> tuple[str, dict]:
    """``start every=20 camera=head`` → ("start", {"every": 20, "camera": "head"})。"""
    parts = text.strip().split()
    if not parts:
        return "", {}
    verb = parts[0].lower()
    kwargs: dict = {}
    for tok in parts[1:]:
        if "=" not in tok:
            continue
        k, v = tok.split("=", 1)
        k = k.strip()
        v = v.strip().strip("\"'")
        if k in ("every", "width", "height"):
            try:
                kwargs[k] = int(v)
            except ValueError:
                _log(f"忽略非法 {k}={v}（要整数）")
        elif k in ("depth", "record_joints"):
            kwargs[k] = v.lower() not in ("0", "false", "no", "off")
        else:
            kwargs[k] = v
    return verb, kwargs


def _handle(text: str) -> None:
    verb, kwargs = _parse_cmd(text)
    if not verb:
        return
    lc = _ensure_capture()

    if verb == "start":
        # 控制文件里可以写 out=... 覆盖输出目录
        if "out" in kwargs and "out_dir" not in kwargs:
            kwargs["out_dir"] = kwargs.pop("out")
        kwargs.setdefault("out_dir", "~/fasim_live/pick_live")
        res = lc.arm_capture(**kwargs)
        if res:
            _log(f"已开始采集 → {res['out']}  相机={res['cameras']}  every={res['every']}")
        else:
            _log("开始失败（可能已挂载），用 stop 先结束")
    elif verb == "stop":
        info = lc.stop_capture()
        if info:
            _log(f"已停止，共 {info['frames_saved']} 帧")
    elif verb == "status":
        s = lc._S
        if s.get("armed"):
            _log(f"采集中: {s['saved']} 帧 / {s['step']} 物理步 → {s['out']}")
        else:
            _log("未在采集")
    else:
        _log(f"未知命令 {verb!r}，支持: start / stop / status")


def _on_app_update(_event) -> None:
    """每帧检查控制文件。只在 mtime 变化时处理，开销可忽略。"""
    import os
    from pathlib import Path

    p = Path(CONTROL_FILE).expanduser()
    try:
        if not p.exists():
            _LAST["exists"] = False
            return
        mtime = p.stat().st_mtime
        if _LAST["exists"] and mtime == _LAST["mtime"]:
            return
        _LAST["exists"] = True
        _LAST["mtime"] = mtime
        text = p.read_text(encoding="utf-8", errors="replace").strip()
        if text:
            _log(f"控制文件: {text!r}")
            _handle(text)
        # 读完清空，避免下次启动重复执行
        p.write_text("", encoding="utf-8")
    except Exception as exc:  # noqa: BLE001
        _log(f"读控制文件失败: {exc}")


class PublicExtension(omni.ext.IExt):
    def on_startup(self, ext_id: str) -> None:
        global _APP, _UPDATE_SUB
        import os
        from pathlib import Path

        ctrl = Path(CONTROL_FILE).expanduser()
        ctrl.parent.mkdir(parents=True, exist_ok=True)
        if not ctrl.exists():
            ctrl.write_text("", encoding="utf-8")

        try:
            _APP = omni.kit.app.get_app()
            _UPDATE_SUB = _APP.get_update_event_stream().create_subscription_to_pop(
                _on_app_update, name="fasim.live_capture.control"
            )
        except Exception as exc:  # noqa: BLE001
            _log(f"WARN 监听 app update 失败，控制文件不可用: {exc}")

        _log("=" * 66)
        _log("旁路采集已就绪（文件触发，无需 Script Editor）")
        _log(f"控制文件: {ctrl}")
        _log("")
        _log("  开始采集:")
        _log(f"    echo start > {ctrl}")
        _log(f"    echo 'start every=20 camera=head' > {ctrl}")
        _log("")
        _log("  结束采集:")
        _log(f"    echo stop > {ctrl}")
        _log("=" * 66)

    def on_shutdown(self) -> None:
        global _UPDATE_SUB, _CAPTURE
        try:
            if _CAPTURE is not None and _CAPTURE._S.get("armed"):
                _CAPTURE.stop_capture()
        except Exception:  # noqa: BLE001
            pass
        _UPDATE_SUB = None
        _CAPTURE = None
