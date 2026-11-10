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

    # --- 采什么（camera / joint / ee / object 的任意子集）---
    ALL_WHAT = ["camera", "joint", "ee", "object"]
    if args.what:
        want = [w.strip() for w in args.what.split(",") if w.strip()]
        bad = [w for w in want if w not in ALL_WHAT]
        if bad:
            _log(f"ERROR 未知采集项 {bad}，可选 {ALL_WHAT}")
            return 1
        what = want
    elif args.yes:
        what = list(ALL_WHAT)
    else:
        print("\n采什么（逗号分隔，回车=全部）")
        print("  camera  相机 RGB-D")
        print("  joint   关节角（回放必需）")
        print("  ee      末端位姿（tcp）")
        print("  object  交互物体位姿")
        ans = _ask("选择", ",".join(ALL_WHAT))
        what = [w.strip() for w in ans.split(",") if w.strip()]
        bad = [w for w in what if w not in ALL_WHAT]
        if bad:
            _log(f"ERROR 未知采集项 {bad}")
            return 1
    if "joint" not in what:
        _log("WARN 没选 joint —— 后续不能用 sdg.sh preview 回放自检")

    # --- 相机 ---
    cfg = load_task_config(robot, task, cfg_root)
    cams = list(getattr(cfg, "cameras", None) or [])

    if "camera" not in what:
        camera = ""  # 不采相机
    elif args.camera:
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
    print(f"  采集项   : {','.join(what)}")
    print(f"  相机     : {camera or ('不采' if 'camera' not in what else '全部')}")
    print(f"  频率     : every={every}")
    print(f"  分辨率   : {f'{width}x{height}' if width else '配置默认（仅相机）'}")
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
        "record_joints": "joint" in what,
        "record_ee": "ee" in what,
        "record_objects": "object" in what,
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


def cmd_info(args: argparse.Namespace) -> int:
    """看一个采集会话里有什么（不转换）。"""
    from fasim.replicator.sdg import pack

    try:
        info = pack.summarize(args.session)
    except Exception as exc:  # noqa: BLE001
        _log(f"ERROR {exc}")
        return 1
    print(json.dumps(info, indent=2, ensure_ascii=False))
    return 0


def cmd_pack(args: argparse.Namespace) -> int:
    """采集会话 → episode npz（层2 能读的格式）。"""
    from fasim.replicator.sdg import pack

    try:
        out = pack.pack_episode(
            args.session,
            out_path=args.out or None,
            frame_dt=args.frame_dt or None,
        )
    except Exception as exc:  # noqa: BLE001
        _log(f"ERROR {exc}")
        return 1
    print(f"episode: {out}")
    return 0


def cmd_preview(args: argparse.Namespace) -> int:
    """用层2 的回放路径预览 episode —— 需要 Isaac python。

    支持两种驱动方式：

    - ``--drive joints``（默认）：逐帧 drive 关节角，物理重演。
      不需要 ROS，能验证「关节角录全了没」。
    - ``--drive ee``：把末端位姿逐帧发到运控的 ``/left_target`` /
      ``/right_target``（geometry_msgs/Pose），由 OCS2 自己算关节。
      需要仿真里运控在跑（``ROS=fiveages`` 变体）。这验证的是
      「末端轨迹是否与运控兼容」。

    两者都能出图；``--drive ee`` 时关节由运控闭环控制，更接近真实执行。
    """
    from pathlib import Path as _P

    ep = _P(args.episode).expanduser()
    if not ep.is_file():
        _log(f"ERROR episode 不存在: {ep}")
        return 1

    if args.drive == "ee":
        return _preview_ee(ep, args)

    _log("预览需要 Isaac python 与渲染，正在启动…")
    from fasim.replicator.sdg import renderer as R
    from fasim.replicator.sdg.config_io import load_robot_config, load_task_config

    robot = load_robot_config(args.robot)
    task = load_task_config(args.robot, args.task)
    out = _P(args.out).expanduser() if args.out else ep.parent / "preview"

    from isaacsim import SimulationApp

    app = SimulationApp(launch_config={"headless": not args.gui})
    try:
        res = R.render_episode(
            episode_path=ep,
            robot_config=robot,
            task_config=task,
            output_dir=out,
            app=app,
            start=0,
            end=None,
            lock_base=True,
        )
        _log(f"回放完成: {res.frames_written} 帧 → {out}")
        _log(f"关节 max_err = {res.joint_max_error*1000:.2f} mrad")
        p95 = res.extras.get("joint_error_p95")
        if p95 is not None:
            _log(f"关节 p95_err = {p95*1000:.2f} mrad")
        dev = res.extras.get("object_deviation") or {}
        for k, st in dev.items():
            _log(f"  {k}: mean {st['mean_mm']:.1f}mm p95 {st['p95_mm']:.1f}mm")
        _log(f"产物: {out}")
    finally:
        try:
            app.close()
        except Exception:  # noqa: BLE001
            pass
    return 0


def _preview_ee(ep: Path, args: argparse.Namespace) -> int:
    """``--drive ee``：把末端位姿发给运控，由 OCS2 闭环驱动。"""
    import os
    import time

    # ROS 环境：和 ee_traj.sh 一致
    ros_setup = os.environ.get("ROS_SETUP", "/opt/ros/jazzy/setup.bash")
    ws_setup = os.environ.get("ROS_WS_SETUP", str(Path.home() / "ros2_ws/install/setup.bash"))
    for f in (ros_setup, ws_setup):
        if Path(f).is_file():
            _log(f"提示：请先 source {f}（当前进程无法替你 source）")
    os.environ.setdefault("RMW_IMPLEMENTATION", "rmw_zenoh_cpp")
    os.environ.setdefault("ROS_DISTRO", "jazzy")
    os.environ.setdefault("ROS_AUTOMATIC_DISCOVERY_RANGE", "LOCALHOST")
    zenoh = Path.home() / "fa_w2_ws/zenoh/DEFAULT_RMW_ZENOH_SESSION_CONFIG.json5"
    if zenoh.is_file():
        os.environ.setdefault("ZENOH_SESSION_CONFIG_URI", str(zenoh))

    try:
        import rclpy
        from geometry_msgs.msg import Pose
    except ImportError:
        _log("ERROR 当前 python 没有 rclpy。请用 source 过 ROS 的 python 跑，"
             "或先执行 ./scripts/sdg.sh preview 走 ee_traj.sh 那套环境。")
        return 1

    from fasim.replicator.sdg.episode import Episode

    episode = Episode.load(ep)
    n = len(episode)
    _log(f"episode: {n} 帧，{episode.num_joints} 关节")
    if not np.all(np.isfinite(episode.left_pos)) or not np.all(np.isfinite(episode.right_pos)):
        _log("ERROR 这个 episode 没有末端位姿（left_pos/right_pos 含 NaN）。"
             "采集时要选 ee（sdg.sh capture --what joint,ee）。")
        return 1

    rclpy.init()
    node = rclpy.create_node("sdg_preview_ee")
    pub_l = node.create_publisher(Pose, args.left_topic, 1)
    pub_r = node.create_publisher(Pose, args.right_topic, 1)

    rate_hz = args.rate if args.rate > 0 else (1.0 / float(np.median(np.diff(episode.t)) if n > 1 else 0.05))
    _log(f"以 {rate_hz:.1f} Hz 发到 {args.left_topic} / {args.right_topic}")
    _log("等待订阅者（运控）……最多 10s")
    t0 = time.monotonic()
    while pub_l.get_subscription_count() == 0 and time.monotonic() - t0 < 10:
        rclpy.spin_once(node, timeout_sec=0.1)
    if pub_l.get_subscription_count() == 0:
        _log("WARN 没有订阅者 —— 运控起了吗？（ROS=fiveages 变体）")
    else:
        _log(f"订阅者: {pub_l.get_subscription_count()}")

    def _mk(p: np.ndarray, q: np.ndarray) -> Pose:
        m = Pose()
        m.position.x, m.position.y, m.position.z = (float(x) for x in p[:3])
        m.orientation.w, m.orientation.x, m.orientation.y, m.orientation.z = (
            float(x) for x in q[:4]
        )
        return m

    try:
        period = 1.0 / rate_hz
        t_start = time.monotonic()
        for i in range(n):
            pub_l.publish(_mk(episode.left_pos[i], episode.left_quat[i]))
            pub_r.publish(_mk(episode.right_pos[i], episode.right_quat[i]))
            rclpy.spin_once(node, timeout_sec=0.0)
            if not args.gui:  # GUI 下靠用户 Ctrl+C 控制节奏没意义，仍按时间走
                pass
            target = t_start + (i + 1) * period
            sleep = target - time.monotonic()
            if sleep > 0:
                time.sleep(sleep)
            if i % 50 == 0:
                _log(f"  {i}/{n}")
        _log(f"发完 {n} 帧")
    except KeyboardInterrupt:
        _log("已中断")
    finally:
        node.destroy_node()
        rclpy.shutdown()
    return 0


def cmd_trim(args: argparse.Namespace) -> int:
    """把连续采集切成任务片段，剔除待机段。"""
    from fasim.replicator.sdg import trim

    try:
        report = trim.trim_session(
            args.session,
            threshold=args.threshold,
            pad_sec=args.pad,
            min_len_sec=args.min_len,
            merge_gap_sec=args.merge_gap,
            write=args.write,
        )
    except Exception as exc:  # noqa: BLE001
        _log(f"ERROR {exc}")
        return 1
    if args.json:
        print(json.dumps(report, indent=2, ensure_ascii=False))
    return 0


def cmd_doctor(_args: argparse.Namespace) -> int:
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
    c.add_argument("--what", default="",
                   help="采什么：camera,joint,ee,object 的逗号子集；空=全部")
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
    sub.add_parser("doctor", help="环境自检").set_defaults(func=cmd_doctor)

    i = sub.add_parser("info", help="看采集会话里有什么")
    i.add_argument("session", help="会话目录，如 ~/fasim_data/.../20261110-1121")
    i.set_defaults(func=cmd_info)

    p = sub.add_parser("pack", help="采集会话 → episode npz（层2 格式）")
    p.add_argument("session")
    p.add_argument("--out", default="", help="输出 npz；留空 <session>/episode.npz")
    p.add_argument("--frame-dt", type=float, default=0.0,
                   help="帧间隔（秒）；留空自动推算")
    p.set_defaults(func=cmd_pack)

    v = sub.add_parser("preview", help="用层2 回放路径预览 episode（需 Isaac python）")
    v.add_argument("episode")
    v.add_argument("--robot", default="unitree_g1d")
    v.add_argument("--task", default="sinopec_test_tube")
    v.add_argument("--out", default="")
    v.add_argument("--gui", action="store_true", help="开窗口（默认无头）")
    v.add_argument(
        "--drive", choices=("joints", "ee"), default="joints",
        help="joints=逐帧 drive 关节角（不需要 ROS）；"
        "ee=把末端位姿发给运控 /left_target,/right_target（需要运控在跑）",
    )
    v.add_argument("--left-topic", default="/left_target")
    v.add_argument("--right-topic", default="/right_target")
    v.add_argument("--rate", type=float, default=0.0,
                   help="--drive ee 时的发送频率；留空按 episode 的 dt")
    v.set_defaults(func=cmd_preview)

    t = sub.add_parser("trim", help="把连续采集切成任务片段，剔除待机段")
    t.add_argument("session", help="会话目录")
    t.add_argument("--threshold", type=float, default=0.01,
                   help="关节角变化率阈值 rad/s（默认 0.01）")
    t.add_argument("--pad", type=float, default=1.0, help="每段前后缓冲秒数")
    t.add_argument("--min-len", type=float, default=2.0, help="短于此秒数的段丢弃")
    t.add_argument("--merge-gap", type=float, default=1.5,
                   help="两段间隔短于此秒数则合并")
    t.add_argument("--write", action="store_true", help="真的写出 segment_*.npz")
    t.add_argument("--json", action="store_true", help="额外打印完整 JSON 报告")
    t.set_defaults(func=cmd_trim)

    args = ap.parse_args(argv)
    if not getattr(args, "func", None):
        ap.print_help()
        return 0
    return args.func(args)


if __name__ == "__main__":
    sys.exit(main())
