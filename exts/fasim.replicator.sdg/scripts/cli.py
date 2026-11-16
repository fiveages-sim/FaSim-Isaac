"""``sdg.sh <子命令>`` 的实现 —— 与随 Isaac 自动加载的扩展通信。

为什么单独一个进程
==================
扩展在 Isaac 进程里，终端没法直接 import 它。两者通过 Unix socket 对话::

    sdg.sh capture  --(JSON over unix socket)-->  extension.py --> live_capture

这样用户永远不用碰 Script Editor，也不用 ``echo > control.txt``。

两种用法
========
**日常用交互菜单**（推荐，会记住上次选择）::

    ./scripts/sdg.sh            # 无参数 = 主菜单

**脚本/CI 用子命令**（本篇实现的就是这些）::

    ./scripts/sdg.sh capture --robot <robot> --task <task> --what joint,ee,object
    ./scripts/sdg.sh pack <session> --robot <robot>
    ./scripts/sdg.sh live <episode.npz> --drive joints --wait

机器人与任务的隔离
==================
本文件**不含任何具体机器人的名字**。所有机器人/任务信息来自::

    configs/<robot>/robot.yaml             机器人级（articulation/ee_links/planning…）
    configs/<robot>/tasks/<task>.yaml      任务级（场景/相机/tracked_prims…）

接入新机器人 = 新建一个 ``configs/<new_robot>/`` 目录，不改代码。
只有一个候选时自动选中；多个候选时才交互问。

交互流程示例（旧版 ``capture`` 的子命令入口）::

    $ ./scripts/sdg.sh capture
    机器人
      1) <robot_a>  /World/...
      2) <robot_b>  /World/...
    选择 [1]: 1

    任务
      1) <task_x>  (默认)
    选择 [1]: 1

    采什么（逗号分隔，回车=全部）
      camera  相机 RGB-D
      joint   关节角（回放必需）
      ee      末端位姿（tcp）
      object  交互物体位姿
    选择 [camera,joint,ee,object]:

    相机
      1) <cam_a>   640x480
      2) 全部相机（2 个，开销最大）
    选择 [2]: 2

    采集频率（每 N 个物理步采一帧）
      1) 10  ≈ 20fps 仿真时间  (推荐)
      2) 20  ≈ 10fps 仿真时间  (省开销)
    选择 [1]: 1

    输出目录
      <空格> = <data_root>/<robot>/<task>/run_01
    输入:

    ============================================================
     准备开始采集
      机器人   : <robot>
      任务     : <task>
      采集项   : camera,joint,ee,object
      相机     : 全部（2 个）
      频率     : every=10
      输出     : <data_root>/...
    ============================================================
    确认开始？ [Y/n]:
    [sdg] 已开始采集（2 相机，every=10）
    [sdg] 现在去跑 motion-generation；结束后执行 ./scripts/sdg.sh stop
"""

from __future__ import annotations

import argparse
import json
import os
import socket
import sys
import time
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


def _pick_multi_what(all_what: list[str], default: list[str] | None) -> list[str] | None:
    """交互式多选「采什么」。

    复用 sdg_menu 的 TTY 多选框（方向键 + 空格）。这样 CLI 和菜单的交互一致，
    而不是一个用框框、一个用逗号 —— 后者是本次要修掉的问题。
    """
    desc = {
        "camera": "相机 RGB-D（同时采内参）",
        "joint": "关节角（回放必需）",
        "ee": "末端位姿（tcp 真值 + 指令）",
        "object": "交互物体位姿",
    }
    opts = [(w, f"{w:8s} {desc.get(w, w)}") for w in all_what]
    try:
        import importlib.util
        from pathlib import Path as _P

        menu_path = _P(__file__).with_name("sdg_menu.py")
        spec = importlib.util.spec_from_file_location("_sdg_menu_pick", menu_path)
        mod = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(mod)
        picked = mod.pick_multi("采集项（空格勾选，回车确认）", opts, default=default)
        return [str(x) for x in picked]
    except Exception as exc:  # noqa: BLE001
        # 兜底：极端情况下退回文本输入，不比现在更差
        _log(f"WARN 多选框不可用（{exc}），退回文本输入")
        print("\n采什么（逗号分隔，回车=全部）")
        for i, (w, d) in enumerate(opts, 1):
            print(f"  {i}) {d}")
        ans = _ask("选择", ",".join(str(i + 1) for i in range(len(opts))))
        out = [opts[int(t) - 1][0] for t in ans.split(",")
               if t.strip().isdigit() and 1 <= int(t) <= len(opts)]
        return out or list(all_what)


def _isaac_pids() -> list[int]:
    """找出正在跑的 Isaac Sim 进程（Kit 主进程）。

    用于在 socket 连不上时区分两种完全不同的情况::

        Isaac 活着但扩展没加载  → 等几秒 / 看 [sdg] 日志
        Isaac 已经崩了（残留 socket 文件）→ 重启仿真

    没有这一步时，两种都会报 "连接失败: [Errno 111]"，用户难以判断。
    """
    pids: list[int] = []
    try:
        import subprocess

        out = subprocess.run(
            ["pgrep", "-f", "isaacsim/kit|isaac-sim.sh"],
            capture_output=True, text=True, timeout=5,
        )
        for line in out.stdout.splitlines():
            line = line.strip()
            if line.isdigit():
                pids.append(int(line))
    except Exception:  # noqa: BLE001
        pass
    return pids


def _stale_socket_hint(path: Path) -> str:
    """socket 连不上时给出可操作的说明（附带自动清理建议）。"""
    pids = _isaac_pids()
    if not pids:
        return (
            f"socket 残留但 Isaac 已不在运行（{path}）\n"
            "        → 仿真可能崩溃了。重新 ./run.sh 启动即可；\n"
            "          残留的 socket 会被下次启动自动覆盖，也可以手动删：\n"
            f"          rm -f {path}"
        )
    return (
        f"socket 存在但连不上（Isaac 进程在跑，pid={pids[:3]}）\n"
        "        → 扩展可能还没加载完（等几秒重试），或加载失败。\n"
        "          查 Isaac 终端里有没有 [sdg] 开头的日志。"
    )


def _request(payload: dict, timeout: float = 30.0) -> dict:
    """向扩展发一条指令，返回它的 JSON 响应。

    对 Isaac 崩溃/卡死做了区分：socket 连不上时会探测 Isaac 进程是否还在，
    给出「重启仿真」还是「等扩展加载」的明确指引 —— 两者原来都只是
    ``Errno 111``，用户无法判断。
    """
    path = runtime.socket_path()
    if not path.exists():
        pids = _isaac_pids()
        if pids:
            err = (
                f"socket 不存在: {path}\n"
                f"        Isaac 在跑（pid={pids[:3]}）但扩展还没就绪 —— 等几秒重试；\n"
                "        仍不行就查 Isaac 终端有没有 [sdg] 开头的日志。"
            )
        else:
            err = (
                "Isaac 没有在运行，也没有可用的采集扩展。\n"
                "        先启动仿真：cd ~/FaSim-Isaac && ./run.sh"
            )
        return {"ok": False, "error": err, "kind": "no_socket"}
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
            return {
                "ok": False,
                "kind": "empty_reply",
                "error": "扩展没有返回（采集/仿真可能崩了，看 Isaac 终端）",
            }
        return json.loads(buf.split(b"\n", 1)[0].decode("utf-8", "replace"))
    except socket.timeout:
        return {
            "ok": False,
            "kind": "timeout",
            "error": "扩展响应超时（Isaac 可能卡住/主循环被阻塞）。\n"
                     "        若长时间无响应，看 Isaac 终端是否已崩溃，必要时重启。",
        }
    except OSError as exc:
        return {
            "ok": False,
            "kind": "refused",
            "error": f"连接失败: {exc}\n        " + _stale_socket_hint(path),
        }
    finally:
        try:
            s.close()
        except OSError:
            pass


# ---------------------------------------------------------------- 子命令


def _auto_snapshot_if_needed(robot: str, task: str) -> bool:
    """采集开始前：没有初始位姿快照就自动记录一份。

    为什么放在这里
    --------------
    随机化的 ``place_on`` 依赖「物体快照」来还原「试管在孔里」这个状态。
    没有快照时随机化会失败，但报错发生在**随机化时**，那时仿真已经跑过
    任务、试管被拿走了 —— 这时补记的快照是**错的**（会记下"试管在左手
    或已插孔"的位置），而且很难察觉。

    所以最稳的时机是「刚 Play、物体还在初始位置、任务还没跑」时自动记。
    采集开始前正好是这个时刻。

    Returns:
        True = 已有或记录成功；False = 失败（不阻塞采集，只警告）。
    """
    import json as _json

    from fasim.replicator.sdg.config_io import load_task_config

    try:
        cfg = load_task_config(robot, task)
    except Exception as exc:  # noqa: BLE001
        _log(f"WARN 读任务配置失败，跳过快照检查: {exc}")
        return False

    rnd_cfg = dict(getattr(cfg, "randomize", {}) or {})
    if not rnd_cfg:
        return True   # 没配随机化，不需要快照

    data_dir = str(getattr(cfg, "output_dir", "") or "")
    if not data_dir:
        _log("WARN 任务配置没有 output_dir，跳过快照检查")
        return False

    snap_file = Path(data_dir).expanduser() / "scene_initial_poses.json"
    if snap_file.is_file():
        # 已有快照：不覆盖。快照必须来自「物体在初始位」的时刻，
        # 中途重记只会把它弄脏（比如试管已被拿走）。
        _log(f"初始位姿快照已存在 ({snap_file})")
        return True

    _log("没有初始位姿快照 —— 自动记录一份")
    _log("  时机：采集开始前，物体应在初始位置")
    resp = _request(
        {"cmd": "randomize_snapshot", "robot_key": robot, "task_key": task,
         "data_dir": data_dir},
        timeout=60,
    )
    if not resp.get("ok"):
        _log(f"WARN 自动记录快照失败: {resp.get('error', '未知')}")
        _log("  随机化的 place_on 可能失效；手动补记: "
             "./sdg.sh randomize --snapshot")
        return False

    snap = resp.get("snapshot") or {}
    n = len(snap.get("objects") or snap) if isinstance(snap, dict) else 0
    _log(f"已记录初始位姿快照（{n} 个物体）→ {snap_file}")
    return True


def cmd_capture(args: argparse.Namespace) -> int:
    from fasim.replicator.sdg.config_io import CONFIG_ROOT, load_task_config

    # --- 机器人 ---
    cfg_root = Path(args.config_root) if args.config_root else CONFIG_ROOT
    robots = sorted(
        p.name for p in (cfg_root).iterdir()
        if p.is_dir() and (p / "robot.yaml").is_file()
    )
    if not robots:
        _log(f"ERROR {cfg_root} 下没有机器人配置（需要 <robot>/robot.yaml）")
        return 1
    robot = args.robot or _pick("机器人", [(r, r) for r in robots])

    # --- 任务 ---
    tdir = cfg_root / robot / "tasks"
    tasks = sorted(p.stem for p in tdir.glob("*.yaml")) if tdir.is_dir() else []
    if not tasks:
        _log(f"ERROR 找不到任务配置: {tdir}")
        return 1
    task = args.task or _pick("任务", [(t, t) for t in tasks])

    # --- 载入任务配置（预设要用到 cameras/physics_hz，所以提前）---
    cfg = load_task_config(robot, task, cfg_root)
    cams = list(getattr(cfg, "cameras", None) or [])

    # --- 采集预设：名字一选，参数全锁定 ---
    # 这是保证「同任务多批次数据格式对齐」的主要手段 —— 手选参数容易某次
    # 改了分辨率/every，导致那批数据无法与之前合并（见 capture_profile）。
    preset_name = getattr(args, "preset", "") or ""
    preset: dict = {}
    presets = dict(getattr(cfg, "capture_presets", None) or {})
    if presets and not preset_name and not getattr(args, "yes", False) \
            and not args.what and not args.every:
        # 交互式且未显式给参数 → 让用户选预设
        opts = [(k, f"{k:12s} {v.get('desc', '')}") for k, v in presets.items()]
        opts.append(("__manual__", "(不使用预设，逐项手选)"))
        sel = _pick("采集预设（推荐 —— 保证多批次格式一致）", opts, 1)
        if sel != "__manual__":
            preset_name = sel
    if preset_name:
        if preset_name not in presets:
            _log(f"ERROR 未知预设 {preset_name!r}，可选: {list(presets)}")
            return 1
        preset = dict(presets[preset_name])
        _log(f"使用预设 {preset_name}：{preset.get('desc', '')}")

    # --- 采什么（camera / joint / ee / object 的任意子集）---
    ALL_WHAT = ["camera", "joint", "ee", "object"]
    if args.what:
        want = [w.strip() for w in args.what.split(",") if w.strip()]
        bad = [w for w in want if w not in ALL_WHAT]
        if bad:
            _log(f"ERROR 未知采集项 {bad}，可选 {ALL_WHAT}")
            return 1
        what = want
    elif preset.get("what"):
        what = [str(w) for w in preset["what"] if str(w) in ALL_WHAT]
        if not what:
            _log(f"ERROR 预设 {preset_name} 的 what 非法: {preset.get('what')}")
            return 1
    elif args.yes:
        what = list(ALL_WHAT)
    else:
        what = _pick_multi_what(ALL_WHAT, None)
        if what is None:
            return 0
    if "joint" not in what:
        _log("WARN 没选 joint —— 后续不能用 sdg.sh preview 回放自检")

    # --- 相机 ---
    if "camera" not in what:
        camera = ""  # 不采相机
    elif args.camera:
        camera = args.camera
    elif preset.get("camera"):
        camera = str(preset["camera"])
        _log(f"  相机（来自预设）: {camera}")
    else:
        opts = [(c.name, f"{c.name:14s} {c.resolution[0]}x{c.resolution[1]}") for c in cams]
        if len(cams) > 1:
            opts.append(("all", f"全部相机（{len(cams)} 个，开销最大）"))
        sel = _pick("相机", opts)
        # **必须传 "all" 而不是 ""** —— arm_capture 里 "" 表示"不采"，
        # 传空会让"选全部"变成"一个都不采"（cam02 踩过这个坑）。
        camera = "all" if sel == "all" else sel

    # --- 频率 ---
    # every 的语义是"每 N 个物理步采一帧"，实际帧率 = 物理步频 / every。
    # 物理步频由**场景资产**决定（physxScene:timeStepsPerSecond），任务配置里
    # 用 record.physics_hz 声明。所以这里必须按任务的步频换算档位，
    # 不能写死 "10 ≈ 17fps"（那是某个特定场景的经验值）。
    phys_hz = getattr(cfg, "physics_hz", None)
    if args.every:
        every = args.every
    elif preset.get("every"):
        # 预设直接给了物理步数
        every = max(1, int(preset["every"]))
        _log(f"  every={every}（来自预设）")
    elif preset.get("target_fps"):
        # 预设给的是目标帧率 → 按本任务 physics_hz 反推（换场景也不会错）
        tgt = float(preset["target_fps"])
        if phys_hz:
            every = max(1, int(round(phys_hz / tgt)))
            _log(f"  目标 {tgt:g} fps → every={every}"
                 f"（{phys_hz:g} Hz / {every} = {phys_hz/every:.1f} fps，来自预设）")
        else:
            every = 10
            _log(f"WARN 预设给了 target_fps={tgt:g} 但任务没写 physics_hz，"
                 f"退回 every={every}")
    else:
        if phys_hz:
            hz_note = f"物理步频 {phys_hz:g} Hz（任务配置 record.physics_hz）"
            # 生成档位：按目标帧率反推 every，只保留能整除的（帧率才精确），
            # 整除不了就标注实际帧率 —— 避免出现 "every=7 才能 28.6fps" 这种
            # 反直觉的数字。
            opts = []
            for tgt in (30, 20, 15, 10, 5):
                ev_exact = phys_hz / tgt
                ev = int(round(ev_exact))
                if ev < 1 or ev in {int(o[0]) for o in opts}:
                    continue
                fps = phys_hz / ev
                exact = abs(ev_exact - ev) < 1e-6
                tag = "" if exact else f"  (目标 {tgt}fps 取整)"
                opts.append((str(ev), f"{ev:3d}  约 {fps:5.1f} fps 仿真时间{tag}"))
            if not opts:
                opts = [("1", "1  每个物理步都采")]
        else:
            hz_note = "物理步频未知（任务配置没写 record.physics_hz，帧率取决于场景）"
            opts = [("5", "5   每 5 个物理步 1 帧"),
                    ("10", "10  每 10 个物理步 1 帧  (常用)"),
                    ("20", "20  每 20 个物理步 1 帧")]
        _log(hz_note)
        sel = _pick("采集频率（每 N 个物理步采一帧）", opts)
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
    elif preset.get("width") and preset.get("height"):
        width, height = int(preset["width"]), int(preset["height"])
        _log(f"  分辨率 {width}x{height}（来自预设）")
    else:
        sel = _pick("分辨率", [("cfg", "用配置默认"), ("small", "320x240 (省开销)")])
        width = 320 if sel == "small" else None
        height = 240 if sel == "small" else None

    # --- 深度 ---
    if args.depth is not None:
        depth = args.depth
    elif "depth" in preset:
        depth = bool(preset["depth"])
        _log(f"  深度图 {'开' if depth else '关'}（来自预设）")
    else:
        depth = _pick("深度图", [("y", "要（推荐）"), ("n", "不要（更省）")]) == "y"

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
    if preset_name:
        print(f"  预设     : {preset_name}  ({preset.get('desc', '')})")
    print(f"  采集项   : {','.join(what)}")
    if "camera" not in what:
        cam_desc = "不采"
    elif str(camera).strip().lower() == "all":
        cam_desc = f"全部（{len(cams)} 个）"
    else:
        cam_desc = str(camera)
    print(f"  相机     : {cam_desc}")
    print(f"  频率     : every={every}"
          + (f"（{phys_hz:g} Hz / {every} ≈ {phys_hz/every:.1f} fps 仿真时间）"
             if phys_hz and every else ""))
    print(f"  分辨率   : {f'{width}x{height}' if width else '配置默认（仅相机）'}")
    print(f"  深度     : {'是' if depth else '否'}")
    print(f"  输出     : {out_dir}")
    print("=" * 62)
    if not args.yes and _ask("确认开始？ [Y/n]", "Y").lower() not in ("", "y", "yes"):
        _log("已取消")
        return 0

    _auto_snapshot_if_needed(robot, task)

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
        # 预设名 —— 写进 session.json，方便回溯「这批数据是哪个预设采的」
        "preset": preset_name or "",
    }
    if width and height:
        req["width"], req["height"] = width, height

    resp = _request(req)
    if not resp.get("ok"):
        _log(f"ERROR {resp.get('error', '未知错误')}")
        return 1
    _log(f"已开始采集 → {resp['out']}")
    _log(f"  相机 {resp['cameras']}  every={resp['every']}")
    _print_next_step(resp.get("out", ""))
    return 0


def _print_next_step(out: str) -> None:
    """采集启动后的醒目提示。

    为什么单独抽出来：这一屏是用户**唯一**会盯着看的地方，但原来只是一行
    ``[sdg] 现在去跑 motion-generation`` —— 混在前面几行日志里，第一次用的
    人容易以为"没反应"。所以：

    - 加大分隔线 + 序号，形成明确的"下一步"区块
    - 不绑定具体工具名（motion-generation 只是当前实现，可能是别的编排器）
    - 给出结束采集的做法，避免忘记 stop 导致一直采
    """
    bar = "=" * 62
    print()
    print(bar)
    print("  ✅  采集已启动，正在后台记录")
    print()
    print("  ▶  下一步：去运行你的【轨迹生成 / 任务编排工具】")
    print("     （让机器人把任务跑一遍，采集中会自动记录）")
    print()
    print("  ■  跑完回来：执行 ./scripts/sdg.sh stop")
    print("     （或在本交互菜单里选「结束采集」）")
    if out:
        print()
        print(f"     数据正在写入: {out}")
    print(bar)
    print()


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
    dc = resp.get("done_checker")
    if dc:
        cur = dc.get("last_d_xy_mm")
        cur_s = f"{cur:.1f}mm" if cur is not None else "—"
        state = "✅ 已完成" if dc.get("done") else f"进行中（当前 xy={cur_s}）"
        _log(f"  完成检测: {state}  判据 {dc['object']}→{dc['rack']} "
             f"xy<{dc['xy_tol_mm']:.0f}mm")
        if dc.get("done"):
            _log(f"    命中帧 {dc.get('done_frame')}")
    elif resp.get("done"):
        _log(f"  完成检测: ✅ {resp['done']}")
    return 0


def _load_state() -> dict:
    """读菜单保存的上下文（上次选的机器人/任务）。

    与 ``sdg_menu.py`` 的 STATE_PATH 保持一致；读不到就返回空 dict
    （``motion`` 命令会提示用 --robot/--task 显式指定）。
    """
    import json as _json

    p = Path.home() / ".cache" / "fasim_sdg" / "menu_state.json"
    try:
        return _json.loads(p.read_text(encoding="utf-8"))
    except Exception:  # noqa: BLE001
        return {}


def cmd_motion(args: argparse.Namespace) -> int:
    """一键起编排：从 sdg 侧拉起 motion-generation。

    为什么要从 sdg 起而不是切仓库手敲 —— 见
    :mod:`fasim.replicator.sdg.motion_launch` 的模块 docstring。
    """
    from fasim.replicator.sdg import motion_launch as ml

    # 机器人/任务：显式参数 > 上次采集用的（state）
    robot = str(getattr(args, "robot", "") or "").strip()
    task = str(getattr(args, "task", "") or "").strip()
    if not (robot and task):
        st = _load_state()
        robot = robot or str(st.get("robot") or "")
        task = task or str(st.get("task") or "")
    if not (robot and task):
        _log("ERROR 不知道要跑哪个任务 —— 用 --robot/--task 指定，"
             "或先跑一次 capture 让我记住上下文")
        return 1

    ok, info = ml.preflight(
        repo_arg=getattr(args, "repo", "") or "",
        robot_key=robot,
        task_key=task,
    )
    if not ok:
        return 2

    _log(f"编排目标: {info['sdg_robot']}/{info['sdg_task']} "
         f"→ {info['robot']}/{info['task_key']}")
    _log(f"  仓库 {info['repo']}")
    _log(f"  工作区 {info['workspace']}")
    if info.get("activate"):
        _log(f"  venv {info['activate']}")
    else:
        _log("  WARN 没找到 venv，将用当前 python（可能缺 rclpy）")

    running = ml.is_running()
    if running:
        _log(f"WARN 已有编排在跑（pid {running[:3]}）—— 可能抢同一个 OCS2/关节")

    if getattr(args, "check", False):
        _log("预检通过（--check 模式，未启动）")
        return 0

    # 采集没开就提醒一句：编排照跑，但采不到数据
    st = _request({"cmd": "status"}, timeout=10)
    if not st.get("armed"):
        _log("WARN 当前未采集 —— 编排会照跑，但采不到数据。"
             "先 ./sdg.sh capture 开始采集")

    return ml.run_motion(
        repo_arg=getattr(args, "repo", "") or "",
        robot_key=robot,
        task_key=task,
        scene=str(getattr(args, "scene", "") or "default"),
        background=bool(getattr(args, "background", False)),
        log_path=str(getattr(args, "log", "") or ""),
    )


def cmd_auto(args: argparse.Namespace) -> int:
    """一键数采：随机化 → 采集 → 编排 → 结束 → 裁剪 → 打包。

    详见 :mod:`fasim.replicator.sdg.auto_collect`。
    """
    from fasim.replicator.sdg import auto_collect, motion_launch as ml

    robot = str(getattr(args, "robot", "") or "").strip()
    task = str(getattr(args, "task", "") or "").strip()
    if not (robot and task):
        st = _load_state()
        robot = robot or str(st.get("robot") or "")
        task = task or str(st.get("task") or "")
    if not (robot and task):
        _log("ERROR 需要 --robot 与 --task（或先跑一次 capture 记住上下文）")
        return 1

    # ---- 预检：Isaac 在跑、socket 通、编排仓库找得到 ----
    _log("预检")
    st = _request({"cmd": "status"}, timeout=10)
    if not st.get("ok"):
        _log(f"ERROR Isaac 扩展没响应: {st.get('error')}")
        _log("     先启动仿真: cd ~/FaSim-Isaac && ./run.sh")
        return 2
    _log("   Isaac 扩展就绪")

    ok, info = ml.preflight(robot_key=robot, task_key=task)
    if not ok:
        return 2
    _log(f"   编排 {info['robot']}/{info['task_key']}")
    _log(f"   工作区 {info['workspace']}")

    try:
        ctx = auto_collect._resolve_task_ctx(robot, task)
    except Exception as exc:  # noqa: BLE001
        _log(f"ERROR 读任务配置失败: {exc}")
        return 2
    _log(f"   数据目录 {ctx['data_dir']}")
    _log(f"   随机化 {'有' if ctx['has_randomize'] else '无'}"
         + (f"，快照{'已存在' if ctx['snapshot_file'].is_file() else '待记录'}"
            if ctx["has_randomize"] else ""))

    rounds = max(1, int(getattr(args, "rounds", 1) or 1))

    # 磁盘预检 —— 长跑最硬的限制（每轮约 5G）
    disk_ok, disk_msg = auto_collect.check_disk_space(ctx["data_dir"], rounds)
    _log(f"   {disk_msg}")
    if not disk_ok:
        _log("ERROR 磁盘空间不足，请先清理（旧 run_* 里的 frames/ 最占空间）")
        return 2

    if getattr(args, "dry_run", False):
        _log(f"预检通过（--dry-run，将跑 {rounds} 轮）")
        return 0

    # ---- 跑 ----
    results = auto_collect.run_batch(
        robot_key=robot,
        task_key=task,
        rounds=rounds,
        scene=str(getattr(args, "scene", "") or "default"),
        every=int(getattr(args, "every", 10) or 10),
        camera=str(getattr(args, "camera", "") or "all"),
        head_sec=float(getattr(args, "head", 2.0)),
        tail_sec=float(getattr(args, "tail", 1.5)),
        do_trim=not bool(getattr(args, "no_trim", False)),
        do_pack=not bool(getattr(args, "no_pack", False)),
        stop_on_fail=not bool(getattr(args, "keep_going", False)),
        skip_randomize=bool(getattr(args, "skip_randomize", False)),
        retries=int(getattr(args, "retries", 1) or 0),
        restart_hint_every=int(getattr(args, "restart_hint_every", 5) or 0),
        do_export=not bool(getattr(args, "no_export", False)),
        prune_keep=int(getattr(args, "keep_frames", 8)),
        batch_export=not bool(getattr(args, "export_each_round", False)),
    )

    if getattr(args, "json", False):
        print(json.dumps(results, indent=2, ensure_ascii=False))

    return 0 if all(r.get("ok") for r in results) else 1


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



# ---------------------------------------------------------------- 随机化


def cmd_randomize(args: argparse.Namespace) -> int:
    """在正在运行的仿真里随机化物体位姿（先归位再叠加偏移）。

    两种用法：
      sdg.sh randomize --snapshot        记录初始位姿（第一轮前跑一次）
      sdg.sh randomize                    按任务配置随机化一轮
      sdg.sh randomize --rounds 5         连续随机 5 轮（验证会不会崩）
      sdg.sh randomize --check            跑可行性验证（带报告）

    配置来自任务的 ``randomize:`` 段；命令行 ``--range-x/--range-y/--range-z``
    可以临时覆盖（调试用，不改 yaml）。
    """
    import json as _json

    robot = str(getattr(args, "robot", "") or "")
    task = str(getattr(args, "task", "") or "")
    if not robot or not task:
        _log("ERROR 需要 --robot 与 --task（或用菜单里的上下文）")
        return 1

    # 输出目录：会话数据的父目录（快照存在这里）
    from fasim.replicator.sdg import config_io, runtime as _rt

    try:
        task_cfg = config_io.load_task_config(robot, task)
    except Exception as exc:  # noqa: BLE001
        _log(f"ERROR 读任务配置失败: {exc}")
        return 1
    data_dir = str(getattr(task_cfg, "output_dir", "") or _rt.data_root())

    cfg = dict(getattr(task_cfg, "randomize", {}) or {})
    if not cfg:
        _log(f"ERROR 任务 {robot}/{task} 的 yaml 里没有 randomize: 段")
        _log("       需要至少一段 objects 声明（见 configs/*/tasks/*.yaml 示例）")
        return 1

    # 命令行覆盖范围（调试方便，不改 yaml）
    rx = getattr(args, "range_x", None)
    ry = getattr(args, "range_y", None)
    rz = getattr(args, "range_z", None)
    if any(v is not None for v in (rx, ry, rz)):
        for obj in (cfg.get("objects") or []):
            if obj.get("follow"):
                continue          # follow 的物体不单独随机
            ax = dict(obj.get("axes") or {})
            for name, val in (("x", rx), ("y", ry), ("z", rz)):
                if val is not None:
                    ax[name] = float(val)
            obj["axes"] = ax
        _log(f"已覆盖随机范围: x={rx} y={ry} z={rz}（None 表示用配置值）")

    if getattr(args, "selftest", False):
        resp = _request({"cmd": "randomize", "selftest": True,
                         "data_dir": data_dir, "cfg": cfg}, timeout=60)
        if not resp.get("ok"):
            _log(f"ERROR {resp.get('error', '未知错误')}")
            return 1
        st = resp.get("selftest") or {}
        _log("=== Isaac 回调内的通道自检 ===")
        _log(f"  stage_ok     : {st.get('stage_ok')}")
        _log(f"  usd_read     : {st.get('usd_read')}   ← 局部 TRS（必须可用）")
        _log(f"  usd_world    : {st.get('usd_world')}  ← USD 世界位姿")
        _log(f"  usd_write    : {st.get('usd_write')}  ← USD 写位姿")
        _log(f"  rigid_read   : {st.get('rigid_read')} ← PhysX 刚体读")
        _log(f"  service_read : {st.get('service_read')} ← ROS 服务（回调里预期超时）")
        ok = st.get("usd_read") is not None and st.get("usd_write") is True
        _log(f"\n  ⇒ {'✅ USD 通道可用，随机化应该正常' if ok else '❌ USD 通道不可用 —— 会退回服务并超时'}")
        return 0 if ok else 1

    if getattr(args, "verify_snapshot", False):
        resp = _request({"cmd": "randomize_verify_snapshot", "data_dir": data_dir,
                         "cfg": cfg}, timeout=60)
        if not resp.get("ok"):
            _log(f"ERROR {resp.get('error', '未知错误')}")
            return 1
        probs = resp.get("problems") or []
        if not probs:
            _log("✅ 快照校验通过（与资产关系一致）")
            return 0
        _log(f"❌ 发现 {len(probs)} 个问题：")
        for p in probs:
            _log(f"  - {p}")
        _log("\n建议：重启仿真（物体回到资产原值）→ 重新跑 --snapshot")
        return 1

    if getattr(args, "snapshot", False):
        resp = _request({"cmd": "randomize_snapshot", "data_dir": data_dir, "cfg": cfg},
                        timeout=60)
        if not resp.get("ok"):
            _log(f"ERROR {resp.get('error', '未知错误')}")
            return 1
        snap = resp.get("snapshot") or {}
        objs = snap.get("objects") or {}
        _log(f"已记录 {len(objs)} 个物体的初始位姿 → {data_dir}/scene_initial_poses.json")
        for prim, rec in objs.items():
            pos = rec.get("pos") if rec else None
            _log(f"  {prim}: {[round(v,4) for v in pos] if pos else '读取失败'}")
        return 0

    rounds = int(getattr(args, "rounds", 1) or 1)
    ok_n = 0
    warns: list[str] = []
    for i in range(rounds):
        resp = _request({"cmd": "randomize", "data_dir": data_dir, "cfg": cfg},
                        timeout=120)
        if not resp.get("ok"):
            _log(f"ERROR 第 {i+1} 轮失败: {resp.get('error', '未知错误')}")
            warns.extend([str(resp.get("error"))])
            continue
        ok_n += 1
        w = resp.get("warnings") or []
        warns.extend(w)
        applied = resp.get("applied") or []
        _log(f"第 {i+1}/{rounds} 轮: {len(applied)} 个物体, seed={resp.get('seed')}"
             + (f", {len(w)} 条告警" if w else ""))
        for a in applied:
            _log(f"    {a['prim']}: offset={a.get('offset')}"
                 + (f" verify_err={a.get('verify_err_m')}m" if "verify_err_m" in a else ""))
        for msg in w:
            _log(f"    WARN {msg}")

    _log(f"完成 {ok_n}/{rounds} 轮，告警 {len(warns)} 条")
    if warns:
        _log("提示：出现告警说明物体被物理改写/碰撞。建议收窄范围或加大 --settle-steps")
    return 0 if ok_n == rounds else 1


def cmd_randomize_check(args: argparse.Namespace) -> int:
    """可行性验证：连续随机 N 轮，看会不会崩/越界。

    结果写到 /tmp/sdg_randomize_check/report.json。
    这是接入自动采集之前的**必经检查**。
    """
    _log("注意：本命令会真的移动物体。建议先保存场景或确认可重载。")
    robot = str(getattr(args, "robot", "") or "")
    task = str(getattr(args, "task", "") or "")
    if not robot or not task:
        _log("ERROR 需要 --robot 与 --task")
        return 1
    # 复用 verify_randomize（它需要在 Isaac 内跑，这里通过 socket 不支持，
    # 所以直接提示用户在 Script Editor 执行）
    _log("可行性验证需要在 **Isaac 的 Script Editor** 里执行（它要逐物理步推进）：")
    _log("")
    _log("    exec(open('/home/fiveages/FaSim-Isaac/exts/fasim.replicator.sdg/"
         "scripts/verify_randomize.py').read())")
    _log("")
    _log("或在 sdg 菜单里选「随机化 → 可行性验证」，它会给出同样的指令。")
    _log(f"（当前上下文: {robot}/{task}；默认 10 轮、settle 60 步）")
    return 0


def cmd_diag_replay(args: argparse.Namespace) -> int:
    """体检一次回放：回放实际到达的末端 vs 编排时到达的末端。

    这是"回放和编排差多少"的**最终判据**：
    - 若 mean < 10mm → 回放忠实复现了编排
    - 若 dz 明显为负（偏下）→ 就是"抓试管偏下"的量化证据
    """
    from fasim.replicator.sdg import fk as fkmod

    path = Path(args.replay).expanduser().resolve()
    if not path.is_file():
        _log(f"ERROR 回放记录不存在: {path}")
        return 1
    src = Path(args.source).expanduser().resolve() if args.source else None
    if src is not None and not src.is_file():
        _log(f"ERROR source episode 不存在: {src}")
        return 1
    try:
        out = fkmod.compare_replay(path, source_episode=src, urdf_path=args.urdf or None)
    except Exception as exc:  # noqa: BLE001
        _log(f"ERROR {exc}")
        return 1

    _log("=" * 62)
    _log(f"回放体检: {path.name}")
    _log("=" * 62)
    sides = out.get("sides") or {}
    if sides:
        _log("\n[1] 回放实际 vs 下发目标（物理稳态误差，天生存在）")
        for side, st in sides.items():
            _log(f"  {side}: mean={st['mean_mm']}mm p95={st['p95_mm']}mm "
                 f"max={st['max_mm']}mm")
            dx, dy, dz = st["worst_delta_xyz_mm"]
            _log(f"    最大 @f{st['worst_frame']}: "
                 f"dx={dx:+.1f} dy={dy:+.1f} dz={dz:+.1f} mm")

    rvr = out.get("replay_vs_recording")
    _log("\n[2] 回放手到的地方 vs 编排手到的地方  ← 关键")
    if not rvr:
        _log("  无法比较（需要 --source episode）")
    else:
        for side, st in rvr.items():
            _log(f"  {side}: mean={st['mean_mm']}mm p95={st['p95_mm']}mm "
                 f"max={st['max_mm']}mm")
            dx, dy, dz = st["worst_delta_xyz_mm"]
            _log(f"    最大 @f{st['worst_frame']}: "
                 f"dx={dx:+.1f} dy={dy:+.1f} dz={dz:+.1f} mm")
        jr = out.get("replay_vs_recording_joint_mrad") or {}
        if jr:
            _log("  关节最大差: "
                 + "  ".join(f"{k}={v}mrad" for k, v in list(jr.items())[:4]))

    _log("\n" + "=" * 62)
    if rvr:
        worst = max(st["mean_mm"] for st in rvr.values())
        if worst < 10.0:
            _log(f"结论: 回放复现良好 ✅（末端均值 {worst:.1f}mm）")
        else:
            _log(f"结论: 回放与编排偏差 {worst:.1f}mm ❌")
            for side, st in rvr.items():
                _log(f"  {side} 方向: {st['worst_delta_xyz_mm']} mm (dx,dy,dz)")
    _log("=" * 62)
    return 0


def _explain_missing_path(p: Path, want: str = "episode.npz") -> None:
    """路径不存在时给出可操作的排查提示，而不是干巴巴一句 ERROR。

    实测常见三种错法：
      1) 给的是**会话目录**而不是 episode.npz  → 提示加 /episode.npz
      2) 还没有 pack 过                         → 提示先跑 pack
      3) 路径打错或少了 robot/task 中间层        → 在数据根下搜同名目录并列出
    """
    root = runtime.data_root()

    if p.is_dir():
        ep = p / want
        if ep.is_file():
            _log(f"提示: 那是个目录，实际文件是 {ep}")
            _log(f"      可直接用: {ep}")
        else:
            _log(f"提示: {p} 是采集会话目录，但里面还没有 {want}")
            _log(f"      先跑:  ./scripts/sdg.sh pack {p} --robot <robot_key>")
            frames = list((p / "frames").glob("frame_*.npz")) \
                if (p / "frames").is_dir() else []
            if frames:
                _log(f"      该会话有 {len(frames)} 帧，可以 pack")
        return

    # 路径不存在：在数据根下按名字搜（最常见是漏了 <robot>/<task>/ 中间层）
    name = p.name
    hits: list[Path] = []
    if root.is_dir() and name:
        try:
            for cand in root.rglob(name):
                if cand.is_dir() and (cand / want).is_file():
                    hits.append(cand)
                elif cand.name == want and cand.is_file():
                    hits.append(cand.parent)
                if len(hits) >= 6:
                    break
        except Exception:  # noqa: BLE001
            pass
    # 也搜带 episode.npz 的同名项
    if not hits and root.is_dir() and name:
        try:
            hits = [d for d in root.rglob(f"*{name}*")
                    if d.is_dir() and (d / want).is_file()][:6]
        except Exception:  # noqa: BLE001
            pass

    if hits:
        _log(f"提示: 数据根目录下有同名会话，你可能是漏了中间的 robot/task 层级:")
        for h in hits:
            try:
                rel = h.relative_to(root)
            except ValueError:
                rel = h
            _log(f"      {rel}")
        _log(f"      直接用第一个: ./scripts/sdg.sh diag {hits[0] / want}")
        return

    _log(f"      当前数据根目录: {root}（用 ./scripts/sdg.sh list 看有哪些）")
    _log("      交互菜单（./scripts/sdg.sh）可直接列表选择，不用手输路径")


def _check_episode_path(raw: str) -> Path | None:
    p = Path(raw).expanduser()
    if p.is_file():
        return p
    _log(f"ERROR episode 不存在: {p}")
    _explain_missing_path(p)
    return None


def cmd_list(args: argparse.Namespace) -> int:
    """列出数据根目录下的采集会话（交互菜单也用它）。"""
    sessions = runtime.list_sessions(
        robot=getattr(args, "robot", "") or None,
        task=getattr(args, "task", "") or None,
        with_episode_only=bool(getattr(args, "episode_only", False)),
        limit=int(getattr(args, "limit", 50) or 50),
    )
    if not sessions:
        _log(f"{runtime.data_root()} 下没有采集会话")
        return 1
    import time

    print(f"{'会话':24s} {'机器人':16s} {'任务':22s} {'帧':>5s} "
          f"{'时长':>7s} {'episode':8s} {'相机':>4s}  时间")
    print("-" * 104)
    for s in sessions:
        age = max(0.0, time.time() - s["mtime"])
        when = time.strftime("%m-%d %H:%M", time.localtime(s["mtime"]))
        print(f"{s['name']:24s} {s['robot'] or '-':16s} {s['task'] or '-':22s} "
              f"{s['frames']:5d} {s['duration']:6.1f}s "
              f"{'有' if s['has_episode'] else '无':8s} {len(s['cameras']):4d}  {when}")
    print(f"\n数据根目录: {runtime.data_root()}")
    return 0


def cmd_bench(args: argparse.Namespace) -> int:
    """把 sdg 采集会话算成与话题方案**同口径**的评测指标。

    为什么需要：评测要横向比"内部直采 vs 话题采集"。两边若各算各的，
    数字没法放一起看。本命令输出与
    ``lerobot_ros2/.../bench_record.py`` 的 ``bench_metrics.json`` **同名同义**
    的字段，直接对比。

    对应关系：
        规模     frames / duration_sec / wall_fps / frame_interval_ms
        对齐     align_spread_ms（sdg 天然≈0，因为是同一物理步回调）
        体积     camera_bytes_per_frame / joint_bytes_per_frame / total_mb_raw
        完整性   nan_joint_frames / camera_shapes / joint_count
        发布率   topic_rates_hz —— 话题方案特有，sdg 不适用（记为 null）
    """
    import numpy as np

    session = Path(args.session).expanduser()
    if not session.is_dir():
        _log(f"ERROR 会话目录不存在: {session}")
        _explain_missing_path(session, want="session.json")
        return 1
    frames_dir = session / "frames"
    files = sorted(frames_dir.glob("frame_*.npz")) if frames_dir.is_dir() else []
    if not files:
        _log(f"ERROR 没有帧文件: {frames_dir}")
        return 1

    meta: dict = {}
    sj = session / "session.json"
    if sj.is_file():
        try:
            meta = json.loads(sj.read_text(encoding="utf-8"))
        except Exception:  # noqa: BLE001
            pass

    wall: list[float] = []
    steps: list[int] = []
    cam_bytes: list[int] = []
    joint_bytes: list[int] = []
    nan_joint = 0
    cams: list[str] = []
    joint_names: list[str] = []
    shapes: dict[str, set[str]] = {}

    for f in files:
        try:
            with np.load(f, allow_pickle=False) as z:
                keys = set(z.files)
                if "wall_time" in keys:
                    wall.append(float(z["wall_time"]))
                if "sim_step" in keys:
                    steps.append(int(z["sim_step"]))
                if not cams:
                    cams = sorted(k[:-4] for k in keys if k.endswith("_rgb"))
                if not joint_names and "joint_names" in keys:
                    joint_names = [str(x) for x in z["joint_names"]]
                cb = 0
                for c in cams:
                    k = f"{c}_rgb"
                    if k in keys:
                        a = np.asarray(z[k])
                        cb += int(a.nbytes)
                        shapes.setdefault(c, set()).add(str(tuple(a.shape)))
                cam_bytes.append(cb)
                if "joint_pos" in keys:
                    q = np.asarray(z["joint_pos"])
                    joint_bytes.append(int(q.nbytes))
                    if not np.all(np.isfinite(q)):
                        nan_joint += 1
        except Exception:  # noqa: BLE001
            continue

    duration = (wall[-1] - wall[0]) if len(wall) >= 2 else float(
        meta.get("duration_sec", 0) or 0)
    interval = np.diff(wall) if len(wall) >= 2 else np.array([0.0])
    phys = np.diff(steps) if len(steps) >= 2 else np.array([0])
    pdt = float(meta.get("physics_dt", 0) or 0)

    metrics = {
        "scheme": "isaac-internal (sdg)",
        "session": str(session),
        "source_config": str(session / "session.json"),
        "task": meta.get("task_key") or "",
        "target_fps": round(1.0 / (pdt * int(meta.get("every", 1) or 1)), 2) if pdt else None,
        "duration_sec": round(float(duration), 3),
        "frames": len(files),
        "wall_fps": round(len(files) / float(duration), 2) if duration > 0 else None,
        "frame_interval_ms": {
            "median": round(float(np.median(interval)) * 1000, 2),
            "p95": round(float(np.percentile(interval, 95)) * 1000, 2),
            "max": round(float(interval.max()) * 1000, 2),
            "jitter_max_ms": round(float(np.abs(interval - np.median(interval)).max()) * 1000, 2),
        },
        "physics": {
            "physics_steps": int(meta.get("physics_steps", 0) or 0),
            "physics_dt": pdt,
            # 设定值（场景 timeStepsPerSecond）—— 决定时间轴
            "physics_hz_setting": meta.get("physics_hz_setting"),
            "physics_hz_declared": meta.get("physics_hz_declared"),
            # 实际值（墙钟每秒物理步数）—— 只反映性能，不参与时间轴换算
            "physics_hz_wall": meta.get("physics_hz_wall"),
            "steps_per_frame": int(np.median(phys)) if phys.size else None,
            "rtf": round(float(meta.get("physics_steps", 0) or 0) * pdt / duration, 3)
            if (pdt and duration > 0) else None,
        },
        # **关键对比项**：sdg 在同一物理步回调内取全部字段，所以是 0。
        # 话题方案（bench_record.py）会给出实际的毫秒数。
        "align_spread_ms": {
            "median": 0.0, "p95": 0.0, "max": 0.0,
            "note": "内部采集在同一 physics step 回调内取全部字段，天然同刻",
        },
        "topic_rates_hz": None,
        "size": {
            "camera_bytes_per_frame": int(np.mean(cam_bytes)) if cam_bytes else 0,
            "joint_bytes_per_frame": int(np.mean(joint_bytes)) if joint_bytes else 0,
            "total_mb_raw": round((sum(cam_bytes) + sum(joint_bytes)) / 1e6, 2),
            "note": "原始 RGB 未压缩；含 depth 时实际体积更大（本项只算 _rgb）",
        },
        "integrity": {
            "nan_joint_frames": int(nan_joint),
            "camera_shapes": {k: sorted(v) for k, v in shapes.items()},
            "joint_count": len(joint_names),
        },
        "camera_names": cams,
        "joint_names": joint_names,
    }

    out = Path(args.out).expanduser() if args.out else session / "bench_metrics.json"
    out.write_text(json.dumps(metrics, indent=2, ensure_ascii=False), encoding="utf-8")

    print()
    print("=" * 62)
    print("评测指标（内部直采）")
    print("=" * 62)
    print(f"帧数        : {metrics['frames']}  ({metrics['duration_sec']}s, {metrics['wall_fps']} fps)")
    fi = metrics["frame_interval_ms"]
    print(f"帧间隔      : median {fi['median']}ms  p95 {fi['p95']}ms  抖动 {fi['jitter_max_ms']}ms")
    ph = metrics["physics"]
    print(f"物理        : {ph['physics_steps']} 步  dt={ph['physics_dt']}s  "
          f"每帧 {ph['steps_per_frame']} 步  RTF={ph['rtf']}")
    hz_set, hz_wall = ph.get("physics_hz_setting"), ph.get("physics_hz_wall")
    if hz_set or hz_wall:
        print(f"  ├ 设定值  : {hz_set or '?'} Hz（场景 timeStepsPerSecond，决定时间轴）")
        print(f"  └ 实际值  : {hz_wall or '?'} Hz（墙钟，受机器性能限制，不参与换算）")
    print(f"对齐间距    : 0 ms（同一物理步回调，天然同刻）")
    sz = metrics["size"]
    print(f"单帧体积    : 图像 {sz['camera_bytes_per_frame']/1e6:.2f}MB + 关节 {sz['joint_bytes_per_frame']/1e3:.1f}KB")
    print(f"总原始体积  : {sz['total_mb_raw']} MB")
    print(f"完整性      : NaN 关节帧 {nan_joint}  关节数 {len(joint_names)}")
    print()
    print(f"指标已写入: {out}")
    print("=" * 62)
    return 0


def cmd_export(args: argparse.Namespace) -> int:
    """把采集会话导出成 LeRobot 数据集（v3.0）。

    支持批量：``--session`` 给**任务目录**（如
    ``~/fasim_data/unitree_g1d/sinopec_test_tube``）时，会把该任务下所有
    会话合并成**一个数据集的多条 episode** —— 这正是「一个任务、多次采集、
    一个训练集」的用法。

    输出目录名字会自动带上 episode 数，便于区分单条 vs 批量导出。
    """
    from fasim.replicator.sdg import export_lerobot as ex

    sessions = _filter_failed_sessions(_expand_sessions(args.session))
    if not sessions:
        _log(f"ERROR 没找到可用采集会话: {args.session}")
        _explain_missing_path(Path(args.session).expanduser(), want="session.json")
        return 1

    root = Path(args.session).expanduser()
    batch = len(sessions) > 1 or not ((root / "frames").is_dir()
                                      or (root / "session.json").is_file())

    # ---- 批量合并前的参数一致性守门 ----
    # 只有参数一致时才能把多个 run 合成一个数据集；否则 feature 定义冲突、
    # 或时间轴混乱（静默成功，训出来的策略无法解释）。
    if len(sessions) > 1:
        from fasim.replicator.sdg import capture_profile as cp

        rep = cp.compare(sessions)
        print()
        print(cp.format_report(rep))
        print()
        if not rep.ok:
            _log("ERROR 采集参数不一致，已中止（加 --allow-mismatch 可强制合并，"
                 "但训练结果将不可解释）")
            if not getattr(args, "allow_mismatch", False):
                return 1
            _log("WARN --allow-mismatch 已指定，继续合并（后果自负）")

    # 默认输出目录：单会话用会话名；批量用「任务目录名」（如 sinopec_test_tube）
    if args.out:
        out = Path(args.out).expanduser()
    elif batch:
        stem = root.name if root.is_dir() else sessions[0].parent.name
        out = Path.home() / "lerobot_datasets" / f"{stem}_{len(sessions)}eps"
    else:
        out = Path.home() / "lerobot_datasets" / sessions[0].name

    if batch:
        _log(f"批量导出：{len(sessions)} 个会话 → 一个数据集（{len(sessions)} episode）")
        for s in sessions:
            _log(f"  · {s.name}")
        if not getattr(args, "yes", False):
            ans = _ask(f"合并导出到 {out}？ [Y/n]", "Y")
            if ans.lower() not in ("", "y", "yes"):
                _log("已取消")
                return 0

    try:
        res = ex.export_session(
            sessions if batch else sessions[0], out,
            repo_id=args.repo_id or None,
            fps=args.fps or None,
            robot_type=args.robot_type or None,
            task=args.task or None,
            use_videos=not args.no_videos,
            overwrite=bool(args.overwrite),
            lerobot_python=args.lerobot_python or None,
            action_space=getattr(args, "action_space", "absolute") or "absolute",
        )
    except Exception as exc:  # noqa: BLE001
        _log(f"ERROR {exc}")
        return 1
    _log(f"导出完成: {res['out']}")
    _log(f"  {res['frames']} 帧 / {res.get('episodes', 1)} episode  features={res['features']}")
    _log(f"  加载方式: LeRobotDataset(repo_id='{out.name}', root='{res['out']}')")
    return 0


def cmd_export_check(args: argparse.Namespace) -> int:
    """校验导出的 LeRobot 数据集能否被官方 LeRobotDataset 正确加载。

    为什么需要：导出成功 ≠ 数据可用。常见问题：
      - 特征 shape/dtype 不匹配 → 加载时报错
      - stats 缺失 → 训练时归一化出错
      - 图像编码失败 → 帧数为 0
    本命令用**官方 loader** 实际读一遍，并抽查若干帧。
    """
    from fasim.replicator.sdg import export_lerobot as ex

    root = Path(args.dataset).expanduser().resolve()
    if not root.exists():
        _log(f"ERROR 数据集不存在: {root}")
        return 1
    py = Path(args.lerobot_python) if args.lerobot_python else ex.find_lerobot_python()
    if py is None:
        _log("ERROR 找不到能 import lerobot 的 python（设 LE_ROBOT_PYTHON）")
        return 1

    script = f"""
import sys
try:
    from lerobot.datasets.lerobot_dataset import LeRobotDataset
except Exception as exc:
    print(f"IMPORT_FAIL {{exc}}"); sys.exit(3)
import numpy as np
try:
    ds = LeRobotDataset(repo_id={args.repo_id!r}, root={str(root)!r})
except Exception as exc:
    print(f"LOAD_FAIL {{type(exc).__name__}}: {{exc}}"); sys.exit(4)
print(f"OK frames={{len(ds)}}")
print(f"features={{sorted(ds.features)}}")
# 抽查首/中/末帧
idxs = [0, len(ds)//2, len(ds)-1]
bad = 0
for i in idxs:
    try:
        f = ds[i]
        for k, v in f.items():
            if hasattr(v, "shape") and v.shape and not np.isfinite(
                np.asarray(v, dtype=np.float32)).all():
                print(f"NaN {{k}} @{{i}}"); bad += 1
    except Exception as exc:
        print(f"FRAME_FAIL {{i}}: {{exc}}"); bad += 1
print(f"CHECKED {{len(idxs)}} frames, bad={{bad}}")

# ---- 数值健全性：action 不能全零/常数 ----
# 曾经出过事故：ee_absolute 从错误的来源取数据 → 全部填零，
# export-check 依然通过（只验证"能加载"），训练白跑一轮。
# 这里检查 action 的方差，全零/常数直接判失败。
import json as _json
from pathlib import Path as _P
try:
    _info = _json.loads((_P({str(root)!r}) / "meta/info.json").read_text())
    _asp = (_info.get("seed") or {{}}).get("action_space")
except Exception:
    _asp = None
try:
    _acts = []
    for _i in range(min(len(ds), 2000)):
        _a = ds[_i].get("action")
        if _a is not None:
            _acts.append(np.asarray(_a, dtype=np.float32).reshape(-1))
    if _acts:
        _A = np.stack(_acts)
        _std = float(_A.std())
        _absmax = float(np.abs(_A).max())
        print(f"ACTION_STATS n={{len(_A)}} dim={{_A.shape[1]}} "
              f"std={{_std:.6f}} absmax={{_absmax:.6f}}")
        if _absmax < 1e-9:
            print("ACTION_ALL_ZERO —— action 全是 0，训练只会学到输出零！")
            bad += 1
        elif _std < 1e-9:
            print("ACTION_CONSTANT —— action 是常数，没有变化")
            bad += 1
        # 状态也查一下（同样可能采空）
        _obs = []
        for _i in range(min(len(ds), 500)):
            _s = ds[_i].get("observation.state")
            if _s is not None:
                _obs.append(np.asarray(_s, dtype=np.float32).reshape(-1))
        if _obs:
            _O = np.stack(_obs)
            print(f"STATE_STATS  std={{float(_O.std()):.6f}} "
                  f"absmax={{float(np.abs(_O).max()):.6f}}")
            if float(np.abs(_O).max()) < 1e-9:
                print("STATE_ALL_ZERO —— observation.state 全是 0！")
                bad += 1

        # ---- 四元数检查（顺序错位很难发现，只影响姿态不影响位置）----
        #
        # 踩过的坑：fk.py 产出 wxyz（USD 约定），而 action 字段名是
        # left_ee.quat.x/.y/.z/.w（ROS 的 xyzw）。直接按名搬会错位一位，
        # 实际旋转变成"绕 Z 转 180°"，表现为左右臂镜像、夹爪朝正后方扭转，
        # 但**位置全对**，所以 MAE/相关性都很好看，只有真驱动机器人才暴露。
        _an = (_info.get("features", {{}}).get("action", {{}}) or {{}}).get("names") or []
        _qidx = [i for i, n in enumerate(_an) if ".quat." in n]
        if _qidx:
            _Q = _A[:, _qidx]
            _norms = np.linalg.norm(_Q.reshape(len(_Q), -1, 4), axis=2)
            print(f"QUAT_NORM    min={{_norms.min():.6f}} max={{_norms.max():.6f}} "
                  f"mean={{_norms.mean():.6f}}")
            if _norms.min() < 0.99 or _norms.max() > 1.01:
                print("QUAT_NOT_UNIT —— 四元数模长不是 1，可能存在顺序错位")
                bad += 1
            # 每组四元数的 w 分量应是最大绝对值之一（接近 1 表示"近无旋转"，
            # 或至少 |w| 不该恒为 ~0）。若 |w| 极小但 |z| 近 1，很可能还是
            # wxyz 被当成 xyzw 存了。
            # 注意：_Q 的列是 _qidx 的**压缩后**下标，不能用 _an 的全局下标去索引
            # （14 维时 6/13 会越界 → 被外层 except 吞掉，连带跳过下面的夹爪检查）。
            _local = {{n: i for i, n in enumerate([_an[j] for j in _qidx])}}
            _wcols = [_local[n] for n in _local if n.endswith(".quat.w")]
            _zcols = [_local[n] for n in _local if n.endswith(".quat.z")]
            if _wcols and _zcols:
                _wabs = float(np.abs(_Q[:, _wcols]).mean())
                _zabs = float(np.abs(_Q[:, _zcols]).mean())
                print(f"QUAT_W_VS_Z  mean|w|={{_wabs:.6f}} mean|z|={{_zabs:.6f}}")
                if _wabs < 1e-6 and _zabs > 0.9:
                    print("QUAT_SUSPECT_ORDER —— |w|≈0 而 |z|≈1，"
                          "疑似 wxyz 被当成 xyzw（应做一次轮转）")
                    bad += 1

        # ---- 夹爪开关量检查（ee_absolute_gripper）----
        #
        # 夹爪必须是 **0/1 开关量**（对齐运控 target_command），不能是关节位置。
        # 若这里看到夹爪维是连续的小数（量级 ~0.02），说明导出用了关节位置 ——
        # 那样闭合时会停在"刚好碰到管子"，**夹持力不足**（踩过的坑）。
        #
        # 另外夹爪必须**至少出现过一次闭合**，否则任务段里根本没抓东西，
        # 策略学不到抓取（表现为推理时伸出去又抽回）。
        _gidx = [i for i, n in enumerate(_an)
                 if n.endswith("_gripper.pos") or "_gripper." in n]
        if _gidx:
            _G = _A[:, _gidx]
            _gmin, _gmax = float(_G.min()), float(_G.max())
            print(f"GRIPPER      dim={{len(_gidx)}} min={{_gmin:.3f}} max={{_gmax:.3f}}")
            _not_binary = ~np.isclose(_G, 0.0) & ~np.isclose(_G, 1.0)
            if float(_not_binary.mean()) > 0.01:
                print("GRIPPER_NOT_BINARY —— 夹爪维不是 0/1 开关量，"
                      "疑似用了关节位置（闭合会夹不紧）。"
                      "应导出 action_space=ee_absolute_gripper")
                bad += 1
            for _c in range(len(_gidx)):
                _col = _G[:, _c]
                _nclose = int(np.isclose(_col, 0.0).sum())
                print(f"  {{_an[_gidx[_c]]}}: 闭合帧 {{_nclose}}/{{len(_col)}} "
                      f"({{100.0*_nclose/max(len(_col),1):.1f}}%)")
                if _nclose == 0:
                    print(f"GRIPPER_NEVER_CLOSED —— {{_an[_gidx[_c]]}} 全程张开，"
                          "策略学不到抓取动作")
                    bad += 1
    else:
        print("WARN 无法读取 action 列（feature 名不是 action？）")
except Exception as exc:
    print(f"WARN 数值检查跳过: {{exc}}")

sys.exit(0 if bad == 0 else 5)
"""
    import subprocess

    _log(f"用 {py} 加载 {root} …")
    r = subprocess.run([str(py), "-c", script])
    if r.returncode == 0:
        _log("✅ 数据集可被 LeRobotDataset 正常加载")
        return 0
    _log(f"❌ 校验失败（退出码 {r.returncode}）")
    return 1


def cmd_diag(args: argparse.Namespace) -> int:
    """一键体检一个 episode：末端真伪 + 关节口径 + 时间轴 + FK 交叉验证。

    设计目标：**跑完这一条就把问题定位完**，不用来回重启试探。

    检查项
    ------
    1. 末端位姿是动态还是死值（与首帧偏离 + 与 Xform 对照值比对）
    2. 末端值与 FK 的一致性（差多少、偏哪个方向）
    3. 有没有 ``joint_pos_cmd``（回放能否避免稳态误差叠加）
    4. drive 稳态误差 = |实测 − 目标|（解释"偏下"的量级）
    5. 时间轴帧间隔来源是否可信（物理步长 vs 墙钟）
    """
    from fasim.replicator.sdg import fk as fkmod

    import numpy as np

    path = Path(args.episode).expanduser().resolve()
    if not path.is_file():
        _log(f"ERROR episode 不存在: {path}")
        _explain_missing_path(path)
        return 1
    try:
        z = np.load(str(path), allow_pickle=True)
    except Exception as exc:  # noqa: BLE001
        _log(f"ERROR 读 npz 失败: {exc}")
        return 1

    keys = set(z.files)
    names = [str(x) for x in z["joint_names"]] if "joint_names" in keys else []
    n = int(z["joint_pos"].shape[0]) if "joint_pos" in keys else 0
    verdicts: list[tuple[bool, str]] = []

    _log("=" * 62)
    _log(f"体检: {path.name}   {n} 帧")
    _log("=" * 62)

    # ---- 1) 末端是动态还是死值 ----
    _log("\n[1] 末端位姿是否真实")
    for side in ("left", "right"):
        pk, xk = f"{side}_pos", f"{side}_pos_xform"
        if pk not in keys:
            _log(f"  {side}: 缺失")
            verdicts.append((False, f"{side}_pos 缺失"))
            continue
        p = np.asarray(z[pk], dtype=np.float64)
        travel = float(np.linalg.norm(p - p[0], axis=1).max() * 1000)
        line = f"  {side}: travel={travel:7.1f}mm"
        ok = travel >= 10.0
        if xk in keys:
            x = np.asarray(z[xk], dtype=np.float64)
            xt = float(np.linalg.norm(x - x[0], axis=1).max() * 1000)
            d = np.linalg.norm(p - x, axis=1) * 1000
            line += f"  Xform对照 travel={xt:.1f}mm 差={d.mean():.1f}mm"
        _log(line + ("   ✅" if ok else "   ❌ 死值"))
        verdicts.append((ok, f"{side} 末端 travel={travel:.0f}mm"))

    # ---- 2) 与 FK 交叉验证 ----
    _log("\n[2] 末端 vs FK（OCS2 planning URDF 正解）")
    try:
        fkr = fkmod.analyze_episode(path, urdf_path=args.urdf or None)
        v = fkr.get("validate") or {}
        for side, st in (v.get("per_side") or {}).items():
            _log(f"  {side}: 与 FK 差 mean={st.get('mean_mm')}mm "
                 f"max={st.get('max_mm')}mm | FK 行程={st.get('fk_travel_mm')}mm")
        if not v.get("ok"):
            _log(f"  ⚠ {v.get('reason')}")
            verdicts.append((False, "末端与 FK 不一致"))
        else:
            verdicts.append((True, "末端与 FK 一致"))
    except Exception as exc:  # noqa: BLE001
        _log(f"  跳过（{type(exc).__name__}: {exc}）")

    # ---- 3) 关节指令口径 ----
    _log("\n[3] 关节角口径")
    if "joint_pos_cmd" in keys:
        qc = np.asarray(z["joint_pos_cmd"], dtype=np.float64)
        qa = np.asarray(z["joint_pos"], dtype=np.float64)
        e = np.abs(qa - qc)
        k = int(np.argmax(e.max(axis=0)))
        _log(f"  ✅ 有 joint_pos_cmd（下发目标）")
        _log(f"  drive 稳态误差: mean={e.mean()*1000:.1f} "
             f"max={e.max()*1000:.1f} mrad"
             + (f"（最大 @{names[k]}）" if k < len(names) else ""))
        # 用这个误差估算末端偏移量级
        if e.max() > 0.01:
            _log(f"  → 若回放误用实测值，末端会多偏 ≈ {e.max()*700:.0f}mm "
                 f"量级（{e.max()*1000:.0f}mrad × 臂长）")
        verdicts.append((True, "记了 joint_pos_cmd"))
    else:
        _log("  ❌ 没有 joint_pos_cmd —— 回放只能用实测值，"
             "末端会多叠一次稳态误差")
        verdicts.append((False, "缺 joint_pos_cmd，需重采"))

    # ---- 4) 时间轴 ----
    _log("\n[4] 时间轴")
    t = np.asarray(z["t"], dtype=np.float64)
    if len(t) > 1:
        dt = np.diff(t)
        med = float(np.median(dt))
        jitter = float(np.abs(dt - med).max() * 1000)
        _log(f"  dt(median)={med*1000:.2f}ms  抖动 max={jitter:.2f}ms  "
             f"总时长={t[-1]:.2f}s")
        ok = jitter < 1.0
        verdicts.append((ok, f"时间轴抖动 {jitter:.1f}ms"))

    # ---- 5) 末端与交互物体的接近关系（若有物体位姿） ----
    # 判据不能用绝对距离：tcp 通常是**法兰**，实际夹持点在指尖前方几厘米，
    # 所以真抓到物体时法兰离物体中心仍有 10-15cm 属正常。看「接近量」
    # （起始距离 − 最小距离）更可靠 —— 手有没有真的伸过去。
    #
    # 用**第一个被移动的物体**作为参照，不写死具体物体名（任务无关）。
    objs_all = sorted(k for k in keys if k.startswith("obj/") and k.endswith("_pos"))
    ref_obj = None
    ref_travel = 0.0
    for k in objs_all:
        p = np.asarray(z[k], dtype=np.float64)
        g = np.isfinite(p).all(axis=1)
        if g.sum() > 1:
            tv = float(np.linalg.norm(p[g] - p[g][0], axis=1).max() * 1000)
            if tv > ref_travel:
                ref_travel, ref_obj = tv, k
    if ref_obj is not None and ref_travel > 30.0:
        label = ref_obj[len("obj/"):-len("_pos")]
        _log(f"\n[5] 末端与交互物体（参照: {label}，移动 {ref_travel:.0f}mm）")
        tp = np.asarray(z[ref_obj], dtype=np.float64)
        for side in ("left", "right"):
            rk = f"{side}_pos"
            if rk not in keys:
                continue
            rp = np.asarray(z[rk], dtype=np.float64)
            good = np.isfinite(tp).all(axis=1) & np.isfinite(rp).all(axis=1)
            if not good.any():
                continue
            d = np.linalg.norm(rp[good] - tp[good], axis=1)
            i = int(np.argmin(d))
            approach = float(d[0] - d.min()) * 1000
            ok = approach > 50.0
            _log(f"  {side}: 起始 {d[0]*1000:.0f}mm → 最近 {d.min()*1000:.0f}mm"
                 f" @f{i}  接近量 {approach:.0f}mm"
                 + ("   ✅" if ok else "   ⚠ 没明显接近"))
            verdicts.append((ok, f"{side} 接近 {label} {approach:.0f}mm"))
        _log("  注: tcp 通常是法兰，实际夹持点在指尖前方，"
             "真抓到物体时距中心 10-15cm 属正常")

    # ---- 6) 物体位姿（rigid 读取的健全性） ----
    objs = sorted(k for k in keys if k.startswith("obj/") and k.endswith("_pos"))
    if objs:
        _log("\n[6] 交互物体位姿")
        for k in objs:
            label = k[len("obj/"):-len("_pos")]
            p = np.asarray(z[k], dtype=np.float64)
            good = np.isfinite(p).all(axis=1)
            if not good.any():
                _log(f"  {label}: 全 NaN ❌")
                verdicts.append((False, f"{label} 位姿全 NaN"))
                continue
            travel = float(np.linalg.norm(p[good] - p[good][0], axis=1).max() * 1000)
            # 架子类应该是静态的（不该被手带走）；试管会被移动
            static_hint = any(t in label.lower() for t in ("rack", "table", "stand"))
            if static_hint and travel > 50.0:
                _log(f"  {label}: travel={travel:.1f}mm  ⚠ 静止物体被移动了")
                verdicts.append((False, f"{label} 不该移动却动了 {travel:.0f}mm"))
            else:
                _log(f"  {label}: travel={travel:.1f}mm  "
                     f"NaN={int((~good).sum())} 帧  ✅")
                verdicts.append((True, f"{label} travel={travel:.0f}mm"))

    # ---- 7) 相机数据 ----
    frames_dir = Path(args.episode).expanduser().resolve().parent / "frames"
    if frames_dir.is_dir():
        _log("\n[7] 相机数据")
        try:
            from importlib import util as _ilu

            _sp = Path(__file__).resolve().parent / "camera_shots.py"
            _spec = _ilu.spec_from_file_location("camera_shots", _sp)
            if _spec is None or _spec.loader is None:
                raise RuntimeError("无法加载 camera_shots")
            _mod = _ilu.module_from_spec(_spec)
            _spec.loader.exec_module(_mod)
            _files = _mod.load_frame_paths(frames_dir.parent)
            _cams = _mod.available_cameras(_files)
            if not _cams:
                _log("  ⚠ 帧里没有相机图像（采集时没选 camera）")
                verdicts.append((False, "没有相机数据"))
            for _cam in _cams:
                rep = _mod.check_frames(_files, _cam)
                line = (f"  {_cam:12s} {rep['frames']} 帧  "
                        f"RGB 均值 {rep['rgb_mean']}  "
                        f"唯一色 {rep['uniq_colors_median']}")
                if "depth_min" in rep:
                    line += (f"  深度 {rep['depth_min']}~{rep['depth_max']}m "
                             f"无效 {rep['depth_invalid_frac']*100:.2f}%")
                _log(line)
                if rep["warnings"]:
                    for w in rep["warnings"]:
                        _log(f"    ✗ {w}")
                    verdicts.append((False, f"相机 {_cam}: {w}"))
                else:
                    _log("    ✅ 量化检查通过")
                    verdicts.append((True, f"相机 {_cam} 正常"))
                # 内参：优先看 episode（pack 会搬），没有就看 frames/ 里的原始帧
                kk = f"{_cam}_K"
                if kk in keys:
                    K = np.asarray(z[kk], dtype=np.float64)
                    if K.ndim == 3:
                        K = K[0]
                    _log(f"    内参: fx={K[0,0]:.2f} fy={K[1,1]:.2f} "
                         f"cx={K[0,2]:.1f} cy={K[1,2]:.1f}"
                         + ("  ✅" if abs(K[0, 0] - K[1, 1]) / max(K[0, 0], 1) < 0.02
                            else "  ⚠ fx/fy 差异大"))
                    verdicts.append((True, f"相机 {_cam} 内参齐全"))
                else:
                    # episode 里没有 —— 可能 pack 版本旧，去看原始帧
                    f0 = None
                    fd = frames_dir / "frames" if (frames_dir / "frames").is_dir() else frames_dir
                    if fd.is_dir():
                        cand = sorted(fd.glob("frame_*.npz"))
                        f0 = cand[0] if cand else None
                    has_in_frame = False
                    if f0 is not None:
                        try:
                            with np.load(str(f0), allow_pickle=False) as zz:
                                has_in_frame = kk in zz.files
                        except Exception:  # noqa: BLE001
                            pass
                    if has_in_frame:
                        _log(f"    内参: ⚠ 原始帧里有 {kk}，但 episode 里没有 —— "
                             f"重新跑 pack 即可搬过来")
                    else:
                        _log(f"    内参: 缺失（该会话采集时没记 camera_params）")
        except Exception as exc:  # noqa: BLE001
            _log(f"  跳过（{type(exc).__name__}: {exc}）")

    # ---- 总结 ----
    _log("\n" + "=" * 62)
    bad = [r for ok, r in verdicts if not ok]
    if bad:
        _log("结论: 发现问题")
        for r in bad:
            _log(f"  ✗ {r}")
    else:
        _log("结论: 全部通过 ✅")
    _log("=" * 62)
    return 0 if not bad else 2


def cmd_fk(args: argparse.Namespace) -> int:
    """用 FK 算末端真值（离线，不需要 Isaac / 不需要 pinocchio 在 Isaac 里）。

    两种用法：

    - ``sdg.sh fk <episode.npz>``：体检采集数据。用关节角做 FK，检查录制的
      ``left_pos`` / ``right_pos`` 是不是真的动态值（本项目实测过它们恒定
      不变，读到的是 authoring 静态值）。
    - ``sdg.sh fk <xxx.replay_f0.npz> --source <episode.npz>``：体检回放。
      对比「回放实际到达的关节角」与「录制目标」的末端偏差，给出毫米级
      mean/p95/max 和最大偏差帧的方向分量 —— 这才是"手偏 2-3cm"的真度量。
    """
    from fasim.replicator.sdg import fk as fkmod

    try:
        path = args.path
        is_replay = str(path).endswith(".npz") and "replay_f" in str(path)
        if is_replay or args.source:
            out = fkmod.compare_replay(
                path,
                source_episode=args.source or None,
                urdf_path=args.urdf or None,
            )
        else:
            out = fkmod.analyze_episode(path, urdf_path=args.urdf or None)
    except Exception as exc:  # noqa: BLE001
        _log(f"ERROR {exc}")
        return 1

    print(json.dumps(out, indent=2, ensure_ascii=False))

    # 关键结论用中文直说，不用让人去读 JSON
    v = out.get("validate")
    if v is not None and not v.get("ok"):
        _log("!" * 56)
        _log(f"末端位姿采集有问题: {v.get('reason')}")
        _log("→ 这不是回放精度问题，是采集时就没读到动态位姿。")
        _log("  原因：newton kit 关了 PhysX→USD/Fabric 同步"
             "（updateToUsd=false / fabricUpdateTransformations=false）。")
        _log("!" * 56)
    sides = out.get("sides")
    if sides:
        for side, st in sides.items():
            _log(f"末端 {side}: 回放实际 vs 下发目标 mean={st['mean_mm']}mm "
                 f"p95={st['p95_mm']}mm max={st['max_mm']}mm")
            if st.get("worst_frame") is not None:
                dx, dy, dz = st["worst_delta_xyz_mm"]
                _log(f"  最大 @frame {st['worst_frame']}: "
                     f"dx={dx:+.1f} dy={dy:+.1f} dz={dz:+.1f} mm")
        _log("  ↑ 这是物理稳态误差（drive 天生有），不是缺陷")

    rvr = out.get("replay_vs_recording")
    if rvr:
        _log("-" * 56)
        _log("关键判据：回放手到的地方 vs 编排手到的地方")
        for side, st in rvr.items():
            _log(f"  末端 {side}: mean={st['mean_mm']}mm p95={st['p95_mm']}mm "
                 f"max={st['max_mm']}mm")
            dx, dy, dz = st["worst_delta_xyz_mm"]
            _log(f"    最大 @frame {st['worst_frame']}: "
                 f"dx={dx:+.1f} dy={dy:+.1f} dz={dz:+.1f} mm")
        jr = out.get("replay_vs_recording_joint_mrad") or {}
        if jr:
            top = list(jr.items())[:4]
            _log("  关节最大差: " + "  ".join(f"{k}={v}mrad" for k, v in top))
        if all(st["mean_mm"] < 10.0 for st in rvr.values()):
            _log("  → 回放复现良好（末端均值 < 10mm）")
        else:
            _log("  → 回放与编排有明显偏差，见上面的方向分量")
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


def _filter_failed_sessions(sessions: list[Path]) -> list[Path]:
    """剔除采集失败/数据损坏的会话，并打印被剔除的清单。

    判据见 ``capture_profile.is_failed_session``：有失败标记 / 0 帧 /
    三路相机分辨率不一致或缺失。这类数据塞进 LeRobot 会让导出中断
    （实测 43 条在第 33 个 episode 因 wrist 分辨率 720x1280 崩掉）。
    """
    from fasim.replicator.sdg import capture_profile as _cp

    failed = [s for s in sessions if _cp.is_failed_session(s)]
    if not failed:
        return sessions
    names = [s.name for s in failed]
    _log(f"跳过 {len(failed)} 个失败/损坏会话: "
         f"{names[:10]}{'…' if len(names) > 10 else ''}")
    _log("  判据：有 .capture_failed.json / 0 帧 / 三路相机分辨率不一致或缺失")
    return [s for s in sessions if s not in failed]


def _expand_sessions(path_str: str) -> list[Path]:
    """把用户给的路径展开成「一个或多个采集会话目录」。

    支持三种输入：

    1. **单个会话**：含 ``frames/`` 或 ``session.json`` → 原样返回
    2. **任务批次目录**：直下一层有多个会话（如
       ``~/fasim_data/unitree_g1d/sinopec_test_tube/`` 下的 ``run_03``/``run_04``）
       → 返回全部子会话（按名字排序）
    3. **机器人目录**：再下一层是任务（``unitree_g1d/sinopec_test_tube``）
       → 递归收集所有任务下的所有会话

    判定「是会话」的依据：目录里有 ``frames/``、``session.json`` 或
    ``episode.npz`` 之一。这样 run_XX / bench01 这类命名都能认出来，
    不依赖命名约定。

    返回空列表表示没找到。
    """
    root = Path(path_str).expanduser()
    if not root.is_dir():
        return []

    def is_session(d: Path) -> bool:
        return ((d / "frames").is_dir()
                or (d / "session.json").is_file()
                or (d / "episode.npz").is_file())

    if is_session(root):
        return [root]

    # 广度优先收集：先看直下，再看下一层
    direct = sorted(p for p in root.iterdir() if p.is_dir() and is_session(p))
    if direct:
        return direct

    # 机器人目录 → 任务目录 → 会话
    nested: list[Path] = []
    for task_dir in sorted(p for p in root.iterdir() if p.is_dir()):
        nested.extend(sorted(p for p in task_dir.iterdir()
                             if p.is_dir() and is_session(p)))
    return nested


def _resolve_batch(args, label: str) -> list[Path] | None:
    """解析批量目标并打印清单。返回 None 表示应中止。

    自动排除「采集失败留下的空壳」（0 帧 / 有 .capture_failed.json 标记）——
    它们会因 cameras=[] / every=0 触发参数不一致，把正常数据也拦下来。
    """
    from fasim.replicator.sdg import capture_profile as _cp

    sessions = _filter_failed_sessions(_expand_sessions(args.session))
    if not sessions:
        _log(f"ERROR 没找到可用采集会话: {args.session}")
        _explain_missing_path(Path(args.session).expanduser(), want="session.json")
        return None

    if len(sessions) > 1:
        _log(f"批量 {label}：{len(sessions)} 个会话")
        for s in sessions:
            n = len(list((s / "frames").glob("frame_*.npz"))) if (s / "frames").is_dir() else 0
            _log(f"  · {s.name}  ({n} 帧)")
        if getattr(args, "yes", False) is False:
            ans = _ask(f"对以上 {len(sessions)} 个会话执行 {label}？ [Y/n]", "Y")
            if ans.lower() not in ("", "y", "yes"):
                _log("已取消")
                return None
    return sessions


def cmd_pack(args: argparse.Namespace) -> int:
    """采集会话 → episode npz（层2 能读的格式）。

    支持批量：``--session`` 给任务目录（含多个 run）会逐个处理。
    """
    from fasim.replicator.sdg import pack

    sessions = _resolve_batch(args, "pack")
    if sessions is None:
        return 1

    ok = fail = 0
    for s in sessions:
        try:
            out = pack.pack_episode(
                s,
                out_path=(args.out or None) if len(sessions) == 1 else None,
                frame_dt=args.frame_dt or None,
                robot_key=getattr(args, "robot", "") or None,
            )
            if len(sessions) == 1:
                print(f"episode: {out}")
            else:
                _log(f"  ✓ {s.name} → {Path(out).name}")
            ok += 1
        except Exception as exc:  # noqa: BLE001
            _log(f"  ✗ {s.name}: {exc}")
            fail += 1

    if len(sessions) > 1:
        _log(f"pack 完成：成功 {ok}，失败 {fail}")
    return 0 if fail == 0 else 1


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
    """把连续采集切成任务片段，剔除待机段。

    支持批量：``--session`` 给任务目录会逐个处理。
    """
    from fasim.replicator.sdg import trim

    sessions = _resolve_batch(args, "trim")
    if sessions is None:
        return 1

    ok = fail = 0
    reports = []
    for s in sessions:
        try:
            report = trim.trim_session(
                s,
                threshold=args.threshold,
                pad_sec=args.pad,
                min_len_sec=args.min_len,
                merge_gap_sec=args.merge_gap,
                write=args.write,
                mode=args.mode,
                head_sec=args.head,
                tail_sec=args.tail,
                out_name=args.out_name,
            )
            if len(sessions) > 1:
                seg = (report or {}).get("segments") or []
                _log(f"  ✓ {s.name}: {len(seg)} 段")
            reports.append(report)
            ok += 1
        except Exception as exc:  # noqa: BLE001
            _log(f"  ✗ {s.name}: {exc}")
            fail += 1

    if args.json:
        print(json.dumps(reports if len(sessions) > 1 else (reports[0] if reports else {}),
                         indent=2, ensure_ascii=False))
    if len(sessions) > 1:
        _log(f"trim 完成：成功 {ok}，失败 {fail}")
    return 0 if fail == 0 else 1


def cmd_live(args: argparse.Namespace) -> int:
    """在同一 Isaac 实例里回放 episode（不弹新窗口）。"""
    ep = Path(args.episode).expanduser().resolve()
    if not ep.is_file():
        _log(f"ERROR episode 不存在: {ep}")
        _explain_missing_path(ep)
        return 1

    req = {
        "cmd": "live",
        "episode": str(ep),
        "drive": args.drive,
        "speed": args.speed,
        "start": args.start,
        "end": args.end,
        "loop": args.loop,
        "robot_key": args.robot,
        "task_key": args.task,
        "pause_cm": bool(args.pause_cm),
        "home_frames": int(args.home_frames),
        "waist_gain": str(args.waist_gain),
        "joint_api": str(args.joint_api),
        "joint_src": str(args.joint_src),
        "reset_objects": bool(args.reset_objects),
        "restore_cm": bool(getattr(args, "restore_cm", False)),
    }
    if args.drive == "ee":
        req["left_topic"] = args.left_topic
        req["right_topic"] = args.right_topic

    resp = _request(req, timeout=30)
    if not resp.get("ok"):
        _log(f"ERROR {resp.get('error', '未知错误')}")
        return 1
    _log(f"开始回放: {resp['frames']} 帧，drive={resp['drive']}，"
         f"驱动 {resp.get('driven_joints', 0)} 个关节")
    if args.wait:
        _log("等待回放结束（Ctrl+C 中断）…")
        try:
            while True:
                time.sleep(0.5)
                st = _request({"cmd": "live_status"}, timeout=5)
                if not st.get("ok"):
                    kind = st.get("kind", "")
                    if kind in ("refused", "no_socket", "empty_reply", "timeout"):
                        _log("")
                        _log("!" * 62)
                        _log("回放中断：与 Isaac 的连接已断开")
                        _log(f"  {st.get('error', '')}")
                        _log("  回放没有正常结束。检查 Isaac 是否崩溃；"
                             "已写入磁盘的关节记录仍可用。")
                        _log("!" * 62)
                    else:
                        _log(f"WARN 回放状态查询失败: {st.get('error')}")
                    break
                if not st.get("playing"):
                    lr = st.get("last_result") or {}
                    if lr:
                        _log("=" * 50)
                        _log(f"回放结果: {lr.get('frames')} 帧")
                        if "max_err_mrad" in lr:
                            _log(f"  关节 max_err = {lr['max_err_mrad']} mrad")
                            _log(f"  关节 p95_err = {lr['p95_err_mrad']} mrad")
                            _log(f"  驱动关节数  = {lr.get('driven_joints')}")
                        jl = lr.get("joint_log")
                        if jl:
                            _log(f"  关节记录 = {jl}")
                            _log("  → 算末端真值: sdg.sh fk "
                                 f"{jl} --source {lr.get('source_episode', '<episode>')}")
                        per_j = lr.get("err_per_joint_mrad") or {}
                        if per_j:
                            top = list(per_j.items())[:4]
                            _log("  误差最大关节: " + "  ".join(f"{k}={v}mrad" for k, v in top))
                        _log(f"  每次 update = {lr.get('app_dt_ms', st.get('app_dt_ms'))} ms")
                        _log("=" * 50)
                    else:
                        _log("回放已结束（Isaac 侧没留下统计，可能是异常退出）")
                    break
                print(f"  {st['frame']}/{st['total']}  t={st['t']:.2f}s  "
                      f"({st.get('frames_done', 0)} 帧已播)", end="\r", flush=True)
        except KeyboardInterrupt:
            _log("请求停止…")
            _request({"cmd": "live_stop"}, timeout=10)
    else:
        _log("回放已在 Isaac 里开始（不阻塞）。进度: ./scripts/sdg.sh live-status")
    return 0


def cmd_live_status(args: argparse.Namespace) -> int:
    resp = _request({"cmd": "live_status"}, timeout=10)
    if not resp.get("ok"):
        _log(f"ERROR {resp.get('error', '未知错误')}")
        return 1
    if not resp.get("playing"):
        _log("当前没有在回放")
        return 0
    _log(f"回放中: {resp['frame']}/{resp['total']}  t={resp['t']:.2f}s"
         + ("  [暂停]" if resp.get("paused") else ""))
    return 0


def cmd_live_stop(args: argparse.Namespace) -> int:
    resp = _request({"cmd": "live_stop"}, timeout=10)
    if not resp.get("ok"):
        _log(f"ERROR {resp.get('error', '未知错误')}")
        return 1
    _log("已请求停止")
    return 0


def cmd_live_pause(args: argparse.Namespace) -> int:
    resp = _request({"cmd": "live_pause"}, timeout=10)
    _log("已暂停" if resp.get("ok") else f"ERROR {resp.get('error')}")
    return 0 if resp.get("ok") else 1


def cmd_live_resume(args: argparse.Namespace) -> int:
    resp = _request({"cmd": "live_resume"}, timeout=10)
    _log("已继续" if resp.get("ok") else f"ERROR {resp.get('error')}")
    return 0 if resp.get("ok") else 1


def cmd_doctor(_args: argparse.Namespace) -> int:
    """自检：路径、socket、扩展、崩溃残留。"""
    print("=== SDG 数采环境自检 ===")
    print(f"扩展根目录 : {runtime.ext_home()}")
    print(f"  存在      : {(runtime.ext_home() / 'config' / 'extension.toml').is_file()}")
    print(f"数据根目录 : {runtime.data_root()}")

    pids = _isaac_pids()
    print(f"Isaac 进程 : {pids or '未运行'}")

    print(f"运行时目录 : {runtime.runtime_dir()}")
    sock = runtime.socket_path()
    print(f"指令 socket: {sock}")
    print(f"  存在      : {sock.exists()}")
    if sock.exists():
        resp = _request({"cmd": "status"}, timeout=5)
        if resp.get("ok"):
            print(f"  响应      : {resp}")
        else:
            print(f"  异常: {resp.get('error')}")
    elif pids:
        print("  → Isaac 在跑但扩展未就绪：等几秒；或查 Isaac 终端 [sdg] 日志")
    else:
        print("  → 仿真没启动（cd ~/FaSim-Isaac && ./run.sh）")

    # --- 崩溃残留检查 ---
    print("-" * 40)
    print("崩溃残留检查")
    stale = sock.exists() and not pids
    if stale:
        print(f"  ⚠ socket 残留但 Isaac 未运行 —— 上次可能崩溃了")
        print(f"    rm -f {sock}   （重启仿真也会自动覆盖）")
    broken = [s for s in runtime.list_sessions(limit=200) if s.get("incomplete")]
    # 空会话目录：有 frames/ 但一帧都没有（采集中断在开头）。这类目录没有
    # session.json 老版本根本发现不了，所以单独扫 frames/。
    empty: list[Path] = []
    root = runtime.data_root()
    if root.is_dir():
        try:
            for fd in root.rglob("frames"):
                if any(p.startswith(".") for p in fd.relative_to(root).parts):
                    continue
                if not any(fd.glob("frame_*.npz")):
                    empty.append(fd.parent)
        except Exception:  # noqa: BLE001
            pass
    if broken:
        print(f"  ⚠ 有 {len(broken)} 个会话采集未正常结束（status=capturing）:")
        for s in broken[:5]:
            print(f"      {s['name']}  {s['frames']} 帧 —— 可直接 pack 抢救")
    if empty:
        print(f"  ⚠ 有 {len(empty)} 个空会话目录（0 帧，可删）:")
        for p in empty[:5]:
            print(f"      {p}")
        print(f"      删除: rm -rf {' '.join(str(p) for p in empty[:5])}")
    if not (stale or broken or empty):
        print("  ✅ 无异常")
    print("=" * 30)
    return 0


def _default_robot() -> str:
    """默认机器人：只有一个配置目录时就用它，否则空串（强制显式指定）。

    **不写死具体机器人** —— 换场景/换机器人时不该改代码。
    """
    try:
        from fasim.replicator.sdg.config_io import CONFIG_ROOT

        rs = sorted(p.name for p in CONFIG_ROOT.iterdir()
                    if p.is_dir() and (p / "robot.yaml").is_file())
        return rs[0] if len(rs) == 1 else ""
    except Exception:  # noqa: BLE001
        return ""


def _default_task(robot: str) -> str:
    """默认任务：该机器人下只有一个任务时用它。"""
    try:
        from fasim.replicator.sdg.config_io import CONFIG_ROOT

        d = CONFIG_ROOT / (robot or "") / "tasks"
        ts = sorted(p.stem for p in d.glob("*.yaml")) if d.is_dir() else []
        return ts[0] if len(ts) == 1 else ""
    except Exception:  # noqa: BLE001
        return ""


def main(argv: list[str] | None = None) -> int:
    ap = argparse.ArgumentParser(
        prog="sdg.sh",
        description="FaSim 合成数据采集（层2.5 旁路采集 + 层2 离线渲染）",
    )
    sub = ap.add_subparsers(dest="cmd")

    c = sub.add_parser("capture", help="交互式开始采集（推荐）")
    c.add_argument("--robot", default="")
    c.add_argument("--task", default="")
    c.add_argument("--preset", default="",
                   help="采集预设名（任务配置 capture_presets 下定义）。"
                        "用预设可保证多批次数据格式一致")
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

    rnd = sub.add_parser("randomize", help="随机化物体位姿（先归位再叠加偏移）")
    rnd.add_argument("--robot", default="", help="机器人 key（默认用菜单上下文）")
    rnd.add_argument("--task", default="", help="任务 key")
    rnd.add_argument("--snapshot", action="store_true",
                     help="只记录初始位姿（第一轮前跑一次）")
    rnd.add_argument("--selftest", action="store_true",
                     help="自检：在 Isaac 回调内各条读/写通道是否可用（排查随机化不生效）")
    rnd.add_argument("--verify-snapshot", dest="verify_snapshot",
                     action="store_true",
                     help="校验快照是否可信（与 USD 资产关系交叉验证）。"
                          "若快照是在物体已崩飞后记录的，这里会报出来")
    rnd.add_argument("--rounds", type=int, default=1, help="连续随机几轮")
    rnd.add_argument("--range-x", dest="range_x", type=float, default=None,
                     help="临时覆盖 x 半幅（米）")
    rnd.add_argument("--range-y", dest="range_y", type=float, default=None)
    rnd.add_argument("--range-z", dest="range_z", type=float, default=None)
    rnd.set_defaults(func=cmd_randomize)
    sub.add_parser("randomize-check", help="随机化可行性验证（会给出 Script Editor 指令）").set_defaults(
        func=cmd_randomize_check)

    mo = sub.add_parser("motion", help="一键起编排（motion-generation 跑一遍任务）")
    mo.add_argument("--robot", default="", help="机器人 key（默认用上次采集的）")
    mo.add_argument("--task", default="", help="任务 key（默认用上次采集的）")
    mo.add_argument("--scene", default="default", help="编排的场景预设（默认 default）")
    mo.add_argument("--repo", default="", help="编排仓库路径（默认 ~/lerobot_ros2）")
    mo.add_argument("--background", "-b", action="store_true",
                    help="后台跑，立即返回（日志默认 /tmp/motion_generation.log）")
    mo.add_argument("--log", default="", help="后台模式的日志路径")
    mo.add_argument("--check", action="store_true",
                    help="只做预检（定位仓库/venv/映射），不真的跑")
    mo.set_defaults(func=cmd_motion)
    ac = sub.add_parser("auto",
                        help="一键数采：随机化→采集→编排→结束→裁剪→打包")
    ac.add_argument("--robot", default="", help="机器人 key")
    ac.add_argument("--task", default="", help="任务 key")
    ac.add_argument("--rounds", type=int, default=1, help="跑几轮（每轮重新随机化）")
    ac.add_argument("--scene", default="default", help="编排的场景预设")
    ac.add_argument("--every", type=int, default=10, help="每 N 个物理步采一帧")
    ac.add_argument("--camera", default="all", help="相机（all 或逗号分隔名字）")
    ac.add_argument("--head", type=float, default=2.0,
                    help="裁剪时开头保留多少秒 home（默认 2.0）")
    ac.add_argument("--tail", type=float, default=1.5,
                    help="裁剪时结尾多留多少秒等停稳（默认 1.5）")
    ac.add_argument("--no-trim", action="store_true", help="不裁剪（保留完整原始帧）")
    ac.add_argument("--no-pack", action="store_true", help="不打包 episode.npz")
    ac.add_argument("--no-export", action="store_true",
                    help="全程不导出 LeRobot 数据集（此时不会删 frames）")
    ac.add_argument("--export-each-round", action="store_true",
                    help="每轮单独导出一个数据集（默认是跑完所有轮后合并成"
                         "一个多-episode 数据集）")
    ac.add_argument("--keep-frames", type=int, default=8,
                    help="导出后保留多少原始帧用于质监（默认 8；-1=全留，0=全删）")
    ac.add_argument("--skip-randomize", action="store_true",
                    help="不随机化（固定位姿，用于复现问题）")
    ac.add_argument("--keep-going", action="store_true",
                    help="某轮失败后继续跑下一轮（默认中止）")
    ac.add_argument("--retries", type=int, default=1,
                    help="单轮失败重试次数（默认 1；长跑建议保留）")
    ac.add_argument("--restart-hint-every", type=int, default=5,
                    help="每 N 轮提醒重启 Isaac（RTF 会逐轮下降；0=关闭）")
    ac.add_argument("--dry-run", action="store_true",
                    help="只做预检，不真的开始")
    ac.add_argument("--json", action="store_true", help="最后打印 JSON 汇总")
    ac.set_defaults(func=cmd_auto)

    sub.add_parser("status", help="查看采集进度").set_defaults(func=cmd_status)
    sub.add_parser("stop", help="结束采集").set_defaults(func=cmd_stop)
    sub.add_parser("doctor", help="环境自检").set_defaults(func=cmd_doctor)

    ls = sub.add_parser("list", help="列出数据根目录下的采集会话")
    ls.add_argument("--robot", default="", help="只看该机器人")
    ls.add_argument("--task", default="", help="只看该任务")
    ls.add_argument("--episode-only", dest="episode_only", action="store_true",
                    help="只列已 pack 出 episode.npz 的")
    ls.add_argument("--limit", type=int, default=50)
    ls.set_defaults(func=cmd_list)

    ex = sub.add_parser(
        "export",
        help="采集会话 → LeRobot 数据集（v3.0，可直接用 LeRobotDataset 加载）",
    )
    ex.add_argument("session", help="采集会话目录")
    ex.add_argument("--out", default="", help="输出目录（默认 ~/lerobot_datasets/<会话名>）")
    ex.add_argument("--repo-id", default="", help="LeRobot repo_id（默认用会话名）")
    ex.add_argument("--fps", type=float, default=0.0,
                    help="帧率；留空由 physics_dt×every 推")
    ex.add_argument("--robot-type", default="", help="写进 info.json 的机器人类型")
    ex.add_argument("--task", default="", help="任务文本（LeRobot 的 task 字段）")
    ex.add_argument("--no-videos", dest="no_videos", action="store_true",
                    help="图像不编码成 mp4（需 v3 允许 image feature；一般不用）")
    ex.add_argument("--overwrite", action="store_true", help="输出目录已存在时清空")
    ex.add_argument("--lerobot-python", default="",
                    help="能 import lerobot 的 python；留空自动找")
    ex.add_argument("-y", "--yes", action="store_true",
                    help="批量导出时不询问确认（脚本/CI 用）")
    ex.add_argument("--allow-mismatch", dest="allow_mismatch", action="store_true",
                    help="批量合并时允许采集参数不一致（危险：时间轴/feature 会错乱）")
    ex.add_argument(
        "--action-space", dest="action_space", default="absolute",
        choices=["absolute", "delta", "ee_delta", "ee_absolute",
                 "ee_absolute_gripper"],
        help="action 语义：absolute=关节指令（默认，部署无累积误差）"
             " / delta=关节单步增量（对齐同事语义，state[t]+action[t]==state[t+1]）"
             " / ee_delta=末端位姿增量 14 维（与同事 pose 模式完全对齐）"
             " / ee_absolute=末端位姿绝对值 14 维（供 inference.py 发运控）"
             " / ee_absolute_gripper=同上 14 维 + 左右夹爪开关量 2 维"
             "（0=闭合 1=张开，对齐运控 target_command；训练用这个才能抓东西）",
    )
    ex.set_defaults(func=cmd_export)

    exc = sub.add_parser(
        "export-check",
        help="校验导出的 LeRobot 数据集能否被官方 LeRobotDataset 加载",
    )
    exc.add_argument("dataset", help="LeRobot 数据集根目录")
    exc.add_argument("--repo-id", default="check", help="repo_id（加载时任意值即可）")
    exc.add_argument("--lerobot-python", default="",
                     help="能 import lerobot 的 python；留空自动找")
    exc.set_defaults(func=cmd_export_check)

    bn = sub.add_parser(
        "bench",
        help="把采集会话算成评测指标（与话题方案同口径，便于横向对比）",
    )
    bn.add_argument("session", help="采集会话目录")
    bn.add_argument("--out", default="", help="输出 json；默认 <session>/bench_metrics.json")
    bn.set_defaults(func=cmd_bench)

    i = sub.add_parser("info", help="看采集会话里有什么")
    i.add_argument("session", help="会话目录，如 ~/fasim_data/.../20261110-1121")
    i.set_defaults(func=cmd_info)

    p = sub.add_parser("pack", help="采集会话 → episode npz（层2 格式）")
    p.add_argument("session")
    p.add_argument("--out", default="", help="输出 npz；留空 <session>/episode.npz")
    p.add_argument("--frame-dt", type=float, default=0.0,
                   help="帧间隔（秒）；留空自动推算")
    p.add_argument(
        "--robot", default="",
        help="机器人 key，用于读 configs/<robot>/robot.yaml 的 planning 段"
        "（决定反算末端指令位姿用哪份 URDF）。留空自动从会话目录猜",
    )
    p.add_argument("-y", "--yes", action="store_true", help="批量时跳过确认")
    p.set_defaults(func=cmd_pack)

    v = sub.add_parser("preview", help="用层2 回放路径预览 episode（需 Isaac python）")
    v.add_argument("episode")
    v.add_argument("--robot", default=_default_robot())
    v.add_argument("--task", default=_default_task(_default_robot()))
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

    t = sub.add_parser("trim", help="裁掉待机段（数采用 whole 模式：留开头 home + 截到回 home）")
    t.add_argument("session", help="会话目录，或任务目录（批量处理该任务下所有会话）")
    t.add_argument("--mode", choices=("segments", "whole"), default="whole",
                   help="whole=整段保留只裁两头待机（数采默认）；"
                        "segments=找所有活跃段各切一条（一个 session 跑多任务时用）")
    t.add_argument("--threshold", type=float, default=0.01,
                   help="关节角变化率阈值 rad/s（默认 0.01）")
    t.add_argument("--pad", type=float, default=1.0, help="[segments] 每段前后缓冲秒数")
    t.add_argument("--min-len", type=float, default=2.0, help="[segments] 短于此秒数的段丢弃")
    t.add_argument("--merge-gap", type=float, default=1.5,
                   help="[segments] 两段间隔短于此秒数则合并")
    t.add_argument("--head", type=float, default=2.0,
                   help="[whole] 开头保留多少秒静止（home 姿态上下文，默认 2.0）")
    t.add_argument("--tail", type=float, default=1.5,
                   help="[whole] 最后一次运动后多留多少秒等停稳（默认 1.5）")
    t.add_argument("--out-name", default="episode_trimmed",
                   help="[whole] 输出文件名（不含 .npz）")
    t.add_argument("--write", action="store_true", help="真的写出 npz")
    t.add_argument("--json", action="store_true", help="额外打印完整 JSON 报告")
    t.add_argument("-y", "--yes", action="store_true", help="批量时跳过确认")
    t.set_defaults(func=cmd_trim)

    lv = sub.add_parser("live", help="在同一 Isaac 实例里回放 episode（不弹新窗口）")
    lv.add_argument("episode")
    lv.add_argument("--robot", default=_default_robot())
    lv.add_argument("--task", default=_default_task(_default_robot()),
                    help="任务配置 key（用于找 tracked_prims 复位物体）")
    lv.add_argument("--drive", choices=("joints", "ee"), default="joints",
                    help="joints=逐帧 drive 关节角（不需要 ROS）；"
                    "ee=发末端位姿给运控 /left_target,/right_target")
    lv.add_argument("--speed", type=float, default=1.0, help="倍速（ee 驱动时生效）")
    lv.add_argument("--start", type=int, default=0)
    lv.add_argument("--end", type=int, default=0, help="0=到末尾")
    lv.add_argument("--loop", action="store_true", help="循环播放")
    lv.add_argument("--left-topic", default="/left_target")
    lv.add_argument("--right-topic", default="/right_target")
    lv.add_argument("--wait", action="store_true", help="阻塞等待回放结束")
    lv.add_argument(
        "--no-pause-cm", dest="pause_cm", action="store_false", default=True,
        help="不回放时暂停运控控制器。默认会暂停（否则运控会覆盖关节目标，"
        "导致动作时快时慢、关节偏几厘米）",
    )
    lv.add_argument(
        "--home-frames", type=int, default=60,
        help="归位过渡帧数（从当前姿态平滑过渡到第 0 帧，默认 60；"
        "0=不归位，直接从当前姿态跳到第 0 帧，可能把物体撞飞）",
    )
    lv.add_argument(
        "--waist-gain", default="usd",
        help="腰部（LZ_mt/LZ_it/waist_pitch）的 drive gain。"
        "usd=用资产原值 60000/6000（默认，腰部和手臂物理特性不同）；"
        "same=和手臂一样用 100000/10000；"
        "或自定义 K,D[,maxForce] 如 60000,6000,5000",
    )
    lv.add_argument(
        "--joint-api", choices=("target", "teleport"), default="target",
        help="驱动关节的方式。target=set_dof_position_targets（走 drive，"
        "物理正确但有静差）；teleport=set_dof_positions（瞬移，无静差、"
        "逐帧精确，但破坏接触）。用它对比可判断是数据问题还是驱动跟不上",
    )
    lv.add_argument(
        "--joint-src", choices=("cmd", "actual"), default="cmd",
        help="用哪套关节角当回放驱动目标。cmd（默认）=采集时下发给 drive 的"
        "目标 joint_pos_cmd —— 与编排一致，末端不会多偏；actual=实测 "
        "joint_pos，会多叠一次稳态误差（手臂前伸时稳定偏下 2-3cm），"
        "仅用于对照复现问题",
    )
    lv.add_argument(
        "--restore-cm", dest="restore_cm", action="store_true", default=False,
        help="回放结束后把控制权交还运控，并把关节复位到回放前姿态。"
        "默认**不做**（停在最后一帧）—— 交还时运控重新激活会猛拉关节"
        "（机器人抽一下），偶尔还会搞坏控制器状态机。"
        "正常流程是回放完直接关仿真，所以默认关掉。",
    )
    lv.add_argument(
        "--no-reset-objects", dest="reset_objects", action="store_false",
        default=True,
        help="不回放前把交互物体复位到 episode 起点。默认会复位"
        "（否则物体停在上次跑完的位置，手会抓空）",
    )
    lv.set_defaults(func=cmd_live)

    sub.add_parser("live-status", help="回放进度").set_defaults(func=cmd_live_status)
    sub.add_parser("live-stop", help="停止回放").set_defaults(func=cmd_live_stop)
    sub.add_parser("live-pause", help="暂停回放").set_defaults(func=cmd_live_pause)
    sub.add_parser("live-resume", help="继续回放").set_defaults(func=cmd_live_resume)


    fkp = sub.add_parser(
        "fk",
        help="用 FK 算末端真值（离线；诊断采集是否真采到位姿 / 回放偏多少）",
    )
    fkp.add_argument("path", help="episode.npz（采集体检）或 *.replay_fN.npz（回放体检）")
    fkp.add_argument(
        "--source", default="",
        help="回放体检时给原始 episode，用于解 world→base（不给则只在 base 系比较）",
    )
    fkp.add_argument(
        "--urdf", default="",
        help="planning URDF；默认自动找 /tmp/ocs2_ros2/*/planning_urdf/*.urdf（最新）",
    )
    fkp.set_defaults(func=cmd_fk)

    dg = sub.add_parser(
        "diag",
        help="一键体检 episode：末端真伪 + 关节口径 + 时间轴 + FK 交叉验证",
    )
    dg.add_argument("episode", help="episode.npz")
    dg.add_argument(
        "--urdf", default="",
        help="planning URDF；默认自动找 /tmp/ocs2_ros2/*/planning_urdf/*.urdf",
    )
    dg.set_defaults(func=cmd_diag)

    dr = sub.add_parser(
        "fk-diag",
        help="体检回放：回放手到的地方 vs 编排手到的地方（mm 级 + 方向）",
    )
    dr.add_argument("replay", help="回放导出的 *.replay_fN.npz")
    dr.add_argument("--source", default="", help="原始 episode.npz（解 world→base 用）")
    dr.add_argument("--urdf", default="", help="planning URDF；默认自动找")
    dr.set_defaults(func=cmd_diag_replay)

    args = ap.parse_args(argv)
    if not getattr(args, "func", None):
        ap.print_help()
        return 0
    return args.func(args)


if __name__ == "__main__":
    sys.exit(main())
