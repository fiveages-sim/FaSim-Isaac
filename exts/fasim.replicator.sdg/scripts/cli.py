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

    # ---- 5) 末端与试管的关系（若有） ----
    # 判据不能用绝对距离：tcp 是**法兰**，实际夹持点在指尖前方几厘米，所以
    # 抓到试管时法兰离试管中心仍有 10-15cm 是正常的。看「接近量」更可靠 ——
    # 手有没有真的伸过去。
    tube = "obj/test_tube_pos"
    if tube in keys:
        _log("\n[5] 末端与试管")
        tp = np.asarray(z[tube], dtype=np.float64)
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
            verdicts.append((ok, f"{side} 接近试管 {approach:.0f}mm"))
        _log("  注: tcp 是法兰，实际夹持点在指尖前方几厘米，"
             "抓到试管时法兰距中心 10-15cm 属正常")

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


def cmd_live(args: argparse.Namespace) -> int:
    """在同一 Isaac 实例里回放 episode（不弹新窗口）。"""
    ep = Path(args.episode).expanduser().resolve()
    if not ep.is_file():
        _log(f"ERROR episode 不存在: {ep}")
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

    lv = sub.add_parser("live", help="在同一 Isaac 实例里回放 episode（不弹新窗口）")
    lv.add_argument("episode")
    lv.add_argument("--robot", default="unitree_g1d")
    lv.add_argument("--task", default="sinopec_test_tube",
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
