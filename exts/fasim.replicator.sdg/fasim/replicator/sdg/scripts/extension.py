"""扩展启动钩子：随 Isaac Sim 自动加载，通过 **Unix socket** 接受采集指令。

架构位置
========
::

    sdg.sh capture          ← 用户只碰这个（交互式 CLI）
        │  发 JSON 指令
        ▼
    <runtime>/capture.sock  ← Unix domain socket
        │  omni.kit.app 的 update 事件驱动（不开线程处理渲染）
        ▼
    本模块 on_startup → 监听 socket → 调 live_capture
        │
        ▼
    live_capture 挂 physics step 回调 → 采 RGB-D + 关节

为什么用 socket 而不是控制文件
==============================
早期版本用 ``echo start > control.txt`` 轮询文件。缺点是：

- CLI 拿不到结果，用户要去看 Isaac 终端日志才知道成没成功
- 文件残留会误触发
- 延迟最多一帧

socket 让 CLI 能同步拿到 ``{"ok": true, "cameras": [...], "frames": 0}``，
``sdg.sh status`` 也能查到实时帧数。

**兼容**：仍保留文件监听（``<runtime>/control.txt``），老脚本不受影响。

为什么不自己开线程
==================
``omni.kit.app`` 的 update 事件已经在主循环里。socket 用非阻塞 accept，
每次 update 里 ``select`` 一下，开销可忽略。开线程反而要处理 GIL 和
跨线程调 Replicator 的安全问题（会崩）。
"""

from __future__ import annotations

import json
import os
import socket
import threading

import omni.ext
import omni.kit.app

from fasim.replicator.sdg import runtime

#: 采集器实例（延迟导入，避免扩展加载时就去 import replicator）
_CAPTURE = None
_APP = None
_UPDATE_SUB = None

_SRV: dict = {"sock": None, "conn": None, "buf": b""}
_CTRL_LAST = {"exists": False, "mtime": 0.0}


def _log(msg: str) -> None:
    print(f"[sdg] {msg}", flush=True)


def _ensure_capture():
    """延迟导入采集模块（此时 kit 已就绪，replicator 可安全 import）。"""
    global _CAPTURE
    if _CAPTURE is not None:
        return _CAPTURE
    import sys

    ext = runtime.ext_home()
    scripts = ext / "scripts"
    for p in (str(ext), str(scripts)):
        if p not in sys.path:
            sys.path.insert(0, p)

    import live_capture as lc  # noqa: PLC0415

    _CAPTURE = lc
    return lc


# ---------------------------------------------------------------- 指令处理


def _parse_kv(text: str) -> tuple[str, dict]:
    """``start every=20 camera=head`` → ("start", {"every": 20, ...})。"""
    parts = text.strip().split()
    if not parts:
        return "", {}
    verb = parts[0].lower()
    kwargs: dict = {}
    for tok in parts[1:]:
        if "=" not in tok:
            continue
        k, v = tok.split("=", 1)
        k, v = k.strip(), v.strip().strip("\"'")
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


def _dispatch(req: dict) -> dict:
    """执行一条指令，返回可 JSON 序列化的结果。

    所有异常都在这里收口 —— 扩展绝不能被一条坏指令搞崩。
    """
    verb = str(req.get("cmd", "")).strip().lower()
    if not verb:
        return {"ok": False, "error": "缺少 cmd"}

    try:
        lc = _ensure_capture()
    except Exception as exc:  # noqa: BLE001
        return {"ok": False, "error": f"加载采集模块失败: {exc}"}

    try:
        if verb == "start":
            # 允许 CLI 传 out_dir；没传就用默认 <data>/<robot>/<task>/<时间戳>
            if "out" in req and "out_dir" not in req:
                req["out_dir"] = req.pop("out")
            req.pop("cmd", None)
            res = lc.arm_capture(**req)
            if not res:
                return {"ok": False, "error": "已在采集中，先 stop"}
            return {"ok": True, **res}

        if verb == "stop":
            info = lc.stop_capture()
            if not info:
                return {"ok": False, "error": "当前未采集"}
            return {"ok": True, **info}

        if verb == "status":
            s = lc._S
            if s.get("armed"):
                return {
                    "ok": True,
                    "armed": True,
                    "frames": s["saved"],
                    "physics_steps": s["step"],
                    "errors": s.get("errors", 0),
                    "out": str(s["out"]) if s["out"] else "",
                    "cameras": [c["name"] for c in s["cameras"]],
                    "every": s["every"],
                    "what": s.get("what", []),
                }
            return {"ok": True, "armed": False}

        if verb in ("live", "play"):
            # 在同一实例里回放 episode（非阻塞，靠 update 推进）
            from fasim.replicator.sdg import live_replay

            ep = str(req.pop("episode", "") or "")
            if not ep:
                return {"ok": False, "error": "缺少 episode 路径"}
            req.pop("cmd", None)
            res = live_replay.play_episode(ep, **req)
            return res

        if verb == "live_stop":
            from fasim.replicator.sdg import live_replay

            live_replay.stop()
            return {"ok": True}

        if verb == "live_pause":
            from fasim.replicator.sdg import live_replay

            live_replay.pause()
            return {"ok": True}

        if verb == "live_resume":
            from fasim.replicator.sdg import live_replay

            live_replay.resume()
            return {"ok": True}

        if verb == "live_status":
            from fasim.replicator.sdg import live_replay

            return {"ok": True, **live_replay.status()}

        return {"ok": False, "error": f"未知指令 {verb!r}（支持 start/stop/status）"}
    except Exception as exc:  # noqa: BLE001
        import traceback

        traceback.print_exc()
        return {"ok": False, "error": f"{type(exc).__name__}: {exc}"}


def _handle_text(text: str) -> dict:
    """文本指令（兼容旧的控制文件写法）。"""
    verb, kwargs = _parse_kv(text)
    return _dispatch({"cmd": verb, **kwargs})


# ---------------------------------------------------------------- socket


def _start_server() -> bool:
    path = runtime.socket_path()
    try:
        runtime.ensure_runtime_dir()
        if path.exists():
            # 残留的旧 socket（上次异常退出）：能连上说明有别的实例在用
            probe = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
            probe.settimeout(0.2)
            try:
                probe.connect(str(path))
                probe.close()
                _log(f"WARN socket 已被占用，跳过监听: {path}")
                return False
            except OSError:
                path.unlink(missing_ok=True)
            finally:
                try:
                    probe.close()
                except OSError:
                    pass

        srv = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
        srv.bind(str(path))
        srv.listen(4)
        srv.setblocking(False)
        os.chmod(path, 0o600)
        _SRV["sock"] = srv
        _log(f"socket 已监听: {path}")
        return True
    except OSError as exc:
        _log(f"WARN 无法创建 socket（{exc}），退回仅文件触发")
        return False


def _pump_socket() -> None:
    """每帧调用：accept 新连接、读一行、回一行。非阻塞。"""
    srv = _SRV.get("sock")
    if srv is None:
        return
    import select

    try:
        # 先处理已有连接的数据
        conn = _SRV.get("conn")
        if conn is not None:
            try:
                chunk = conn.recv(4096)
            except (BlockingIOError, InterruptedError):
                return
            except OSError:
                _close_conn()
                return
            if not chunk:
                _close_conn()
                return
            _SRV["buf"] += chunk
            if b"\n" not in _SRV["buf"]:
                return
            line, _, rest = _SRV["buf"].partition(b"\n")
            _SRV["buf"] = rest
            text = line.decode("utf-8", "replace").strip()
            if not text:
                return
            try:
                req = json.loads(text)
                resp = _dispatch(req) if isinstance(req, dict) else _handle_text(text)
            except json.JSONDecodeError:
                resp = _handle_text(text)
            try:
                conn.sendall((json.dumps(resp, ensure_ascii=False) + "\n").encode())
            except OSError:
                pass
            _close_conn()
            return

        # 没有活动连接时尝试 accept
        r, _, _ = select.select([srv], [], [], 0)
        if srv in r:
            conn, _ = srv.accept()
            conn.setblocking(False)
            _SRV["conn"] = conn
            _SRV["buf"] = b""
    except (OSError, ValueError):
        _close_conn()


def _close_conn() -> None:
    conn = _SRV.get("conn")
    _SRV["conn"] = None
    _SRV["buf"] = b""
    if conn is not None:
        try:
            conn.close()
        except OSError:
            pass


# ---------------------------------------------------------------- 文件兼容


def _pump_control_file() -> None:
    """兼容旧方式：``echo start > <runtime>/control.txt``。"""
    p = runtime.runtime_dir() / "control.txt"
    try:
        if not p.exists():
            _CTRL_LAST["exists"] = False
            return
        mtime = p.stat().st_mtime
        if _CTRL_LAST["exists"] and mtime == _CTRL_LAST["mtime"]:
            return
        _CTRL_LAST["exists"] = True
        _CTRL_LAST["mtime"] = mtime
        text = p.read_text(encoding="utf-8", errors="replace").strip()
        if text:
            _log(f"控制文件: {text!r}")
            resp = _handle_text(text)
            _log(f"  结果: {resp}")
        p.write_text("", encoding="utf-8")
    except OSError as exc:
        _log(f"读控制文件失败: {exc}")


def _on_app_update(_event) -> None:
    _pump_socket()
    _pump_control_file()


# ---------------------------------------------------------------- 扩展


class PublicExtension(omni.ext.IExt):
    def on_startup(self, ext_id: str) -> None:
        global _APP, _UPDATE_SUB

        sock_ok = _start_server()
        try:
            _APP = omni.kit.app.get_app()
            _UPDATE_SUB = _APP.get_update_event_stream().create_subscription_to_pop(
                _on_app_update, name="fasim.sdg.capture_ipc"
            )
        except Exception as exc:  # noqa: BLE001
            _log(f"WARN 监听 app update 失败: {exc}")

        ctrl = runtime.runtime_dir() / "control.txt"
        _log("=" * 68)
        _log("数采扩展已就绪")
        _log(f"  扩展根目录 : {runtime.ext_home()}")
        _log(f"  数据根目录 : {runtime.data_root()}")
        _log(f"  指令 socket: {runtime.socket_path()}"
             + ("" if sock_ok else "  (不可用，仅支持文件)"))
        _log(f"  兼容文件   : {ctrl}")
        _log("")
        _log("  推荐用法（另开终端）:")
        _log("    ./scripts/sdg.sh capture     # 交互式开始采集")
        _log("    ./scripts/sdg.sh status      # 看进度")
        _log("    ./scripts/sdg.sh stop        # 结束")
        _log("=" * 68)

    def on_shutdown(self) -> None:
        global _UPDATE_SUB, _CAPTURE
        try:
            if _CAPTURE is not None and _CAPTURE._S.get("armed"):
                _CAPTURE.stop_capture()
        except Exception:  # noqa: BLE001
            pass
        _close_conn()
        srv = _SRV.get("sock")
        if srv is not None:
            try:
                srv.close()
            except OSError:
                pass
            _SRV["sock"] = None
        try:
            runtime.socket_path().unlink(missing_ok=True)
        except OSError:
            pass
        _UPDATE_SUB = None
        _CAPTURE = None
