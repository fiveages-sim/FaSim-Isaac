"""``sdg.sh capture/status/stop`` 的实现 —— 与随 Isaac 自动加载的扩展通信。

为什么单独一个进程
==================
扩展在 Isaac 进程里，终端没法直接 import 它。两者通过 Unix socket 对话::

    sdg.sh capture  --(JSON over unix socket)-->  extension.py --> live_capture

这样用户永远不用碰 Script Editor，也不用 ``echo > control.txt``。

交互流程（对齐 ./run.sh / motion-generation 的风格）
====================================================
::

    $ ./scripts/sdg.sh capture
    机器人
      1) unitree_g1d  (默认)
    选择 [1]: 1

    任务
      1) sinopec_test_tube  (默认)
    选择 [1]: 1

    相机
      1) head             640x480
      2) wrist_left      640x480
      3) wrist_right     640x480
      4) 全部
    选择 [1]: 4

    采集频率（每 N 个物理步采一帧）
      1) 10  ≈ 17fps  (推荐)
      2) 20  ≈  9fps  (省开销)
      3) 40  ≈  4fps  (最省)
      4) 自定义
    选择 [1]: 1

    分辨率
      1) 640x480  (配置默认)
      2) 320x240  (省开销)
    选择 [1]: 2

    深度图
      1) 要 (默认)   2) 不要
    选择 [1]: 1

    输出目录
      <空格> = /home/xxx/fasim_data/unitree_g1d/sinopec_test_tube/20261110-1030
    输入:

    ============================================================
     准备开始采集
      机器人   : unitree_g1d
      任务     : sinopec_test_tube
      相机     : head,wrist_left,wrist_right
      频率     : every=10
      分辨率   : 320x240
      深度     : 是
      输出     : /home/xxx/fasim_data/...
    ============================================================
    确认开始？ [Y/n]:
    [sdg] 已开始采集（3 相机，every=10）
    [sdg] 现在去跑 motion-generation；结束后执行 ./scripts/sdg.sh stop
"""

from __future__ import annotations

import argparse
import json
import socket
import sys
from pathlib import Path

SDG_EXT = Path(__file__).resolve().parents[1]
if str(SDG_EXT) not in sys.path:
    sys.path.insert(0, str(SDG_EXT))

from fasim.replicator.sdg import runtime  # noqa: E402


def _log(msg: str) -> None:
    print(f"[sdg] {msg}", flush=True)


def _ask(prompt: str, default: str = "") -> str:
    try:
        ans = input(f"{prompt} [{default}]: " if default else f"{prompt}: ")
    except EOFError:
        print()
        return default
    return ans.strip() or default


def _pick(title: str, options: list[tuple[str, str]], default: int = 1) -> str:
    """options: [(value, 说明)]。返回选中的 value。"""
    print(f"\n{title}")
    for i, (_val, desc) in enumerate(options, 1):
        mark = "  (默认)" if i == default else ""
        print(f"  {i}) {desc}{mark}")
    while True:
        ans = _ask("选择", str(default))
        if ans.isdigit() and 1 <= int(ans) <= len(options):
            return options[int(ans) - 1][0]
        _log(f"无效选择 {ans!r}")


def _request(payload: dict, timeout: float = 30.0) -> dict:
    """向扩展发一条指令，返回它的 JSON 响应。"""
    path = runtime.socket_path()
    if not path.exists():
        return {
            "ok": False,
            "error": (
                f"socket 不存在: {path}\n"
                "        仿真启动了吗？扩展会随 Isaac 自动加载。\n"
                "        若刚启动，等几秒；仍不行就查 Isaac 终端有没有 [sdg] 开头的日志。"
            ),
        }
    s = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
    s.settimeout(timeout)
    try:
        s.connect(str(path))
        s.sendall((json.dumps(payload, ensure_ascii=False) + "\n").encode())
        buf = b""
        while b"\n" not in buf:
            chunk = s.recv(65536)
            if not chunk:
                break
            buf += chunk
        if not buf.strip():
            return {"ok": False, "error": "扩展没有返回（采集可能崩了，看 Isaac 终端）"}
        return json.loads(buf.split(b"\n", 1)[0].decode("utf-8", "replace"))
    except socket.timeout:
        return {"ok": False, "error": "扩展响应超时（Isaac 可能卡住或未响应）"}
    except OSError as exc:
        return {"ok": False, "error": f"连接失败: {exc}"}
    finally:
        try:
            s.close()
        except OSError:
            pass


# ---------------------------------------------------------------- 子命令


def cmd_capture(args: argparse.Namespace) -> int:
    from fasim.replicator.sdg.config_io import CONFIG_ROOT, load_task_config

    # --- 机器人 ---
    cfg_root = Path(args.config_root) if args.config_root else CONFIG_ROOT
    robots = sorted(
        p.name for p in (cfg_root).iterdir()
        if p.is_dir() and (p / "robot.yaml").is_file()
    ) or ["unitree_g1d"]
    robot = args.robot or _pick("机器人", [(r, r) for r in robots])

    # --- 任务 ---
    tdir = cfg_root / robot / "tasks"
    tasks = sorted(p.stem for p in tdir.glob("*.yaml")) if tdir.is_dir() else []
    if not tasks:
        _log(f"ERROR 找不到任务配置: {tdir}")
        return 1
    task = args.task or _pick("任务", [(t, t) for t in tasks])

    # --- 相机 ---
    try:
        cfg = load_task_config(robot, task, cfg_root)
        cams = list(getattr(cfg, "cameras", None) or [])
    except Exception as exc:  # noqa: BLE001
        _log(f"ERROR 读任务配置失败: {exc}")
        return 1
    if not cams:
        _log("ERROR 任务配置里没有相机")
        return 1

    if args.camera:
        camera = args.camera
    else:
        opts = [(c.name, f"{c.name:14s} {c.resolution[0]}x{c.resolution[1]}") for c in cams]
        if len(cams) > 1:
            opts.append(("all", "全部相机（开销最大）"))
        sel = _pick("相机", opts)
        camera = "" if sel == "all" else sel

    # --- 频率 ---
    if args.every:
        every = args.every
    else:
        sel = _pick(
            "采集频率（每 N 个物理步采一帧）",
            [("10", "10  ≈ 17fps  (推荐)"),
             ("20", "20  ≈  9fps  (省开销)"),
             ("40", "40  ≈  4fps  (最省)")],
        )
        every = int(sel) if sel.isdigit() else 0
        if every == 0:
            while True:
                ans = _ask("输入每帧物理步数", "10")
                if ans.isdigit() and int(ans) > 0:
                    every = int(ans)
                    break
                _log("要正整数")

    # --- 分辨率 ---
    if args.width and args.height:
        width, height = args.width, args.height
    else:
        sel = _pick("分辨率", [("cfg", "用配置默认"), ("small", "320x240 (省开销)")])
        width = 320 if sel == "small" else None
        height = 240 if sel == "small" else None

    # --- 深度 ---
    depth = args.depth if args.depth is not None else (
        _pick("深度图", [("y", "要（推荐）"), ("n", "不要（更省）")]) == "y"
    )

    # --- 输出 ---
    default_out = runtime.default_session_dir(robot, task)
    if args.out:
        out_dir = str(Path(args.out).expanduser().resolve())
    else:
        print(f"\n输出目录\n  <空格> = {default_out}")
        ans = _ask("输入", "")
        out_dir = str(Path(ans).expanduser().resolve()) if ans else str(default_out)

    # --- 确认 ---
    print("\n" + "=" * 62)
    print(" 准备开始采集")
    print(f"  机器人   : {robot}")
    print(f"  任务     : {task}")
    print(f"  相机     : {camera or '全部'}")
    print(f"  频率     : every={every}")
    print(f"  分辨率   : {f'{width}x{height}' if width else '配置默认'}")
    print(f"  深度     : {'是' if depth else '否'}")
    print(f"  输出     : {out_dir}")
    print("=" * 62)
    if not args.yes and _ask("确认开始？ [Y/n]", "Y").lower() not in ("", "y", "yes"):
        _log("已取消")
        return 0

    req = {
        "cmd": "start",
        "robot_key": robot,
        "task_key": task,
        "camera": camera,
        "every": every,
        "depth": bool(depth),
        "out_dir": out_dir,
    }
    if width and height:
        req["width"], req["height"] = width, height

    resp = _request(req)
    if not resp.get("ok"):
        _log(f"ERROR {resp.get('error', '未知错误')}")
        return 1
    _log(f"已开始采集 → {resp['out']}")
    _log(f"  相机 {resp['cameras']}  every={resp['every']}")
    _log("现在去跑 motion-generation；结束后执行 ./scripts/sdg.sh stop")
    return 0


def cmd_status(args: argparse.Namespace) -> int:
    resp = _request({"cmd": "status"}, timeout=10)
    if not resp.get("ok"):
        _log(f"ERROR {resp.get('error', '未知错误')}")
        return 1
    if not resp.get("armed"):
        _log("当前未在采集")
        return 0
    _log(f"采集中: {resp['frames']} 帧 / {resp['physics_steps']} 物理步")
    _log(f"  相机 {resp['cameras']}  every={resp['every']}  错误 {resp['errors']}")
    _log(f"  输出 {resp['out']}")
    return 0


def cmd_stop(args: argparse.Namespace) -> int:
    resp = _request({"cmd": "stop"}, timeout=60)
    if not resp.get("ok"):
        _log(f"ERROR {resp.get('error', '未知错误')}")
        return 1
    n = resp.get("frames_saved", 0)
    err = resp.get("capture_errors", 0)
    _log(f"已停止：{n} 帧，错误 {err}")
    if n == 0:
        _log("WARN 一帧都没采到 —— 仿真在跑吗（timeline PLAY）？")
    _log(f"产物: {resp.get('out', '')}")
    return 0


def cmd_doctors(_args: argparse.Namespace) -> int:
    """自检：路径、socket、扩展是否就绪。"""
    print("=== SDG 数采环境自检 ===")
    print(f"扩展根目录 : {runtime.ext_home()}")
    print(f"  存在      : {(runtime.ext_home() / 'config' / 'extension.toml').is_file()}")
    print(f"数据根目录 : {runtime.data_root()}")
    print(f"运行时目录 : {runtime.runtime_dir()}")
    sock = runtime.socket_path()
    print(f"指令 socket: {sock}")
    print(f"  存在      : {sock.exists()}")
    if sock.exists():
        resp = _request({"cmd": "status"}, timeout=5)
        print(f"  响应      : {resp}")
    else:
        print("  → 仿真没启动，或扩展没加载（跑 ./init.sh 选 isaac_local 安装扩展）")
    print("=" * 30)
    return 0


def main(argv: list[str] | None = None) -> int:
    ap = argparse.ArgumentParser(
        prog="sdg.sh",
        description="FaSim 合成数据采集（层2.5 旁路采集 + 层2 离线渲染）",
    )
    sub = ap.add_subparsers(dest="cmd")

    c = sub.add_parser("capture", help="交互式开始采集（推荐）")
    c.add_argument("--robot", default="")
    c.add_argument("--task", default="")
    c.add_argument("--camera", default="", help="相机名，逗号分隔；空=全部")
    c.add_argument("--every", type=int, default=0, help="每 N 个物理步采一帧")
    c.add_argument("--width", type=int, default=0)
    c.add_argument("--height", type=int, default=0)
    c.add_argument("--depth", dest="depth", action="store_true", default=None)
    c.add_argument("--no-depth", dest="depth", action="store_false")
    c.add_argument("--out", default="", help="输出目录；留空用默认")
    c.add_argument("--config-root", default="")
    c.add_argument("-y", "--yes", action="store_true", help="跳过确认")
    c.set_defaults(func=cmd_capture)

    sub.add_parser("status", help="查看采集进度").set_defaults(func=cmd_status)
    sub.add_parser("stop", help="结束采集").set_defaults(func=cmd_stop)
    sub.add_parser("doctor", help="环境自检").set_defaults(func=cmd_doctors)

    args = ap.parse_args(argv)
    if not getattr(args, "func", None):
        ap.print_help()
        return 0
    return args.func(args)


if __name__ == "__main__":
    sys.exit(main())
