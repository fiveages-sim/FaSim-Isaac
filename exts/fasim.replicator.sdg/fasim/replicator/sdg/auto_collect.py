"""一键数采 —— 随机化 → 采集 → 编排 → 结束 → 裁剪 → 打包。

为什么需要
==========
单轮采集原来要手动执行 6 步（且容易漏）：

  1. ``sdg.sh randomize``           换物体位置
  2. ``sdg.sh capture``             开始采集
  3. 等采集真的起来（socket 就绪）
  4. ``sdg.sh motion``              跑编排
  5. ``sdg.sh stop``                编排跑完手动停
  6. ``sdg.sh trim`` + ``pack``     裁待机段、转 episode 格式

漏任何一步都会出问题：忘了 record snapshot → 随机化用错基准；忘了 stop
→ 一直往磁盘写；忘了 trim → 训练集里一半是机器人站着不动的画面。

所以把 6 步收成一个命令，中途任一步失败就停下并保留现场（数据不删，
方便排查）。

流程与状态机
============
::

    [预检]  Isaac 在跑？socket 通？机器人/任务匹配？venv 找到？
        ↓
    [快照]  没有初始位姿快照就自动记（关键：必须在物体初始位时做）
        ↓
    [随机化] socket randomize  → 失败则中止（物体位置不对，采了也没用）
        ↓
    [开始采集] socket start    → 拿到 out_dir
        ↓
    [跑编排] 子进程 motion-generation（复用 motion_launch）
        ↓
    [结束采集] socket stop     → 写 session.json（含 stop_reason）
        ↓
    [裁剪] trim whole 模式      → 留开头 home + 截到回 home
        ↓
    [打包] pack → episode.npz

为什么每轮只跑一条（不做 num_runs 内循环）
==========================================
编排自己支持 ``num_runs``，但那会把多轮挤在一个 session 里，随机化只能
在**开头做一次**。而数采的核心诉求恰恰是「每轮换物体位置」。所以这里
在**外层**循环：每轮 = 一次完整流程（随机化 → 采 → 停）。

这样每轮得到独立目录 ``run_NNN/``，且各自有 ``episode_trimmed.npz``，
下游导出时不用再拆分。
"""

from __future__ import annotations

import json
import os
import subprocess
import sys
import time
from pathlib import Path

# 与 cli.py / sdg_menu.py 共用同一 socket 协议
DEFAULT_TIMEOUT_RANDOMIZE = 120.0
DEFAULT_TIMEOUT_START = 60.0
DEFAULT_TIMEOUT_STOP = 180.0


def _log(msg: str) -> None:
    print(f"[auto] {msg}", flush=True)


def _request(payload: dict, timeout: float = 30.0) -> dict:
    """向 Isaac 扩展发指令（与 cli.py 的 _request 同协议）。

    单独实现一份而不是 import cli：cli 是脚本（不是包），从模块里 import
    会触发它顶层的 argparse 设置等副作用。
    """
    import socket as _sock

    from fasim.replicator.sdg import runtime

    path = runtime.socket_path()
    if not path.exists():
        return {"ok": False, "error": "socket 不存在（Isaac 未运行或扩展未就绪）"}

    s = _sock.socket(_sock.AF_UNIX, _sock.SOCK_STREAM)
    s.settimeout(timeout)
    try:
        # 先序列化：Path 之类不可序列化的对象如果混进请求，json.dumps 会抛
        # TypeError，但那个错误发生在发送前、且栈指向 json —— 很难看出是
        # 哪个字段的问题。这里提前收口并指出字段名。
        try:
            body = json.dumps(payload, ensure_ascii=False)
        except TypeError as exc:
            bad = [
                k for k, v in payload.items()
                if not isinstance(v, (str, int, float, bool, type(None), list, dict))
            ]
            raise TypeError(
                f"{exc}；可疑字段: {bad or '(嵌套在内)'} —— "
                "路径类请先 str()"
            ) from None
        s.connect(str(path))
        s.sendall((body + "\n").encode())
        buf = b""
        while b"\n" not in buf:
            chunk = s.recv(65536)
            if not chunk:
                break
            buf += chunk
        if not buf.strip():
            return {"ok": False, "error": "扩展没有返回（Isaac 可能卡住）"}
        return json.loads(buf.split(b"\n", 1)[0].decode("utf-8", "replace"))
    except Exception as exc:  # noqa: BLE001
        return {"ok": False, "error": f"{type(exc).__name__}: {exc}"}
    finally:
        try:
            s.close()
        except OSError:
            pass


def check_disk_space(data_dir: str, rounds: int, per_round_gb: float = 5.0) -> tuple[bool, str]:
    """预检磁盘够不够跑 N 轮。返回 (ok, 说明)。

    为什么必须查：每轮约 4.3~5.5G（3 相机 640x480 + 深度，600~800 帧），
    50 轮就是 250G。跑到一半磁盘满会导致 np.savez 静默失败（帧数在涨但
    文件写不进去），比直接报错更难发现。

    实际用量会随帧数浮动，这里用保守估计 + 20% 余量。
    """
    import shutil

    try:
        target = Path(data_dir).expanduser()
        # 目录可能还不存在 —— 向上找第一个存在的祖先
        while not target.exists() and target != target.parent:
            target = target.parent
        usage = shutil.disk_usage(target)
    except Exception as exc:  # noqa: BLE001
        return True, f"无法查磁盘（{exc}），跳过检查"

    need_gb = rounds * per_round_gb * 1.2
    free_gb = usage.free / (1024 ** 3)
    msg = (f"磁盘剩余 {free_gb:.0f}G，预计需要 {need_gb:.0f}G"
           f"（{rounds} 轮 × {per_round_gb}G × 1.2 余量）")
    if free_gb < need_gb:
        return False, f"{msg} —— 空间不足"
    return True, msg


def _resolve_task_ctx(robot_key: str, task_key: str) -> dict:
    """读任务配置，拿到 output_dir / randomize / done_when。

    **所有路径统一转 str** —— 这些值最终会进 socket 的 JSON 请求，
    Path 不可序列化。
    """
    from fasim.replicator.sdg import config_io, runtime

    cfg = config_io.load_task_config(robot_key, task_key)
    out_dir = str(getattr(cfg, "output_dir", "") or runtime.data_root())
    return {
        "cfg": cfg,
        "data_dir": out_dir,
        "has_randomize": bool(getattr(cfg, "randomize", None)),
        "snapshot_file": Path(out_dir).expanduser() / "scene_initial_poses.json",
    }


def ensure_snapshot(robot_key: str, task_key: str, ctx: dict) -> bool:
    """确保初始位姿快照存在（与 cli._auto_snapshot_if_needed 同逻辑）。

    独立实现一份是为了 auto_collect 能单独用（不依赖 cli 这层脚本）。
    """
    if not ctx["has_randomize"]:
        return True
    snap_file = ctx["snapshot_file"]
    if snap_file.is_file():
        _log(f"   快照已存在: {snap_file.name}")
        return True

    _log("   没有快照 —— 自动记录（此刻物体应在初始位置）")
    resp = _request(
        {"cmd": "randomize_snapshot", "robot_key": robot_key,
         "task_key": task_key, "data_dir": ctx["data_dir"]},
        timeout=60,
    )
    if not resp.get("ok"):
        _log(f"   ERROR 记录快照失败: {resp.get('error')}")
        return False
    _log("   快照已记录")
    return True


def randomize_once(robot_key: str, task_key: str, ctx: dict) -> tuple[bool, dict]:
    """随机化一轮。返回 (ok, 结果)。"""
    if not ctx["has_randomize"]:
        _log("   任务没配 randomize，跳过")
        return True, {}

    resp = _request(
        {"cmd": "randomize", "robot_key": robot_key, "task_key": task_key,
         "data_dir": ctx["data_dir"]},
        timeout=DEFAULT_TIMEOUT_RANDOMIZE,
    )
    if not resp.get("ok"):
        _log(f"   ERROR 随机化失败: {resp.get('error')}")
        return False, resp
    objs = resp.get("objects") or resp.get("written") or {}
    if isinstance(objs, dict):
        for k, v in list(objs.items())[:4]:
            if isinstance(v, (list, tuple)) and len(v) >= 3:
                _log(f"   {Path(str(k)).name:24s} → "
                     f"[{v[0]:.4f}, {v[1]:.4f}, {v[2]:.4f}]")
    return True, resp


def start_capture(robot_key: str, task_key: str, out_dir: str, every: int = 10,
                  camera: str = "all") -> tuple[bool, str]:
    """开始采集。返回 (ok, 实际输出目录)。"""
    req = {
        "cmd": "start",
        "robot_key": robot_key,
        "task_key": task_key,
        "camera": camera,
        "every": int(every),
        "depth": True,
        # **必须 str**：socket 请求要 json.dumps。runtime.default_session_dir()
        # 返回 Path，直接塞进来会报 "Object of type PosixPath is not JSON
        # serializable"（实测踩过，且报错发生在 Isaac 侧回调里，栈不直观）。
        "out_dir": str(out_dir),
        "record_joints": True,
        "record_ee": True,
        "record_objects": True,
    }
    resp = _request(req, timeout=DEFAULT_TIMEOUT_START)
    if not resp.get("ok"):
        _log(f"   ERROR 开始采集失败: {resp.get('error')}")
        return False, ""
    actual = str(resp.get("out") or out_dir)
    _log(f"   已开始 → {actual}")
    return True, actual


def run_orchestration(robot_key: str, task_key: str, scene: str = "default",
                      motion_launch_mod=None) -> int:
    """跑一遍编排（子进程，阻塞到结束）。返回退出码。"""
    if motion_launch_mod is None:
        from fasim.replicator.sdg import motion_launch as motion_launch_mod

    _log("   启动 motion-generation（阻塞等待跑完）…")
    return motion_launch_mod.run_motion(
        robot_key=robot_key,
        task_key=task_key,
        scene=scene,
        background=False,
    )


def stop_capture() -> tuple[bool, dict]:
    """结束采集。返回 (ok, 结果)。"""
    resp = _request({"cmd": "stop"}, timeout=DEFAULT_TIMEOUT_STOP)
    if not resp.get("ok"):
        _log(f"   ERROR 结束采集失败: {resp.get('error')}")
        return False, resp
    n = resp.get("frames_saved", 0)
    reason = resp.get("stop_reason", "")
    done = resp.get("done") or {}
    _log(f"   已结束：{n} 帧，stop_reason={reason}")
    if done:
        _log(f"   完成判定: xy={done.get('d_xy_mm')}mm dz={done.get('dz_mm')}mm "
             f"tilt={done.get('tilt_deg')}° (帧 {done.get('frame')})")
    return True, resp


def export_to_lerobot(
    session_dir: str,
    out_root: str = "",
    *,
    robot_type: str = "",
    fps: float | None = None,
) -> dict:
    """把会话导出成 LeRobot 数据集（**必须在删 frames 之前做**）。

    为什么 auto 流程必须包含这步
    ---------------------------
    ``export_lerobot`` 强依赖 ``frames/*.npz``（读 ``<cam>_rgb`` 编码成 mp4）。
    删了 frames 就再也导不出训练格式。而 frames 是空间占用的绝对大头
    （每轮 4~5G，其中 RGB+深度占 95%）。

    所以正确顺序是::

        trim → pack → **export lerobot** → 删 frames（留几帧质监）

    这样一套 auto 跑完就直接得到「可训练的数据集 + 极小的 raw 存档」。

    Args:
        out_root: 数据集根目录；默认 ``~/lerobot_datasets/<robot>_<task>``。
            注意 export 的 ``out_dir`` 是**整个数据集**（多 episode 共用一个），
            不是每条一个目录 —— 这里传的是父目录，由 export_session
            自己按 repo_id 建。
        robot_type: 写进 info.json 的 robot_type。
        fps: 默认由 physics_dt × every 推。

    Returns:
        export_session 的返回 dict（含 out/frames/episodes/features）；
        失败时返回 ``{"error": ...}``。
    """
    from fasim.replicator.sdg import export_lerobot as ex

    sess = Path(session_dir).expanduser()
    if out_root:
        out = Path(out_root).expanduser()
    else:
        # 用会话名做数据集名（每个 run 一个独立数据集）
        out = Path.home() / "lerobot_datasets" / sess.name

    try:
        res = ex.export_session(
            sess,
            out,
            robot_type=robot_type or None,
            fps=fps,
            overwrite=True,     # 会话名唯一，重建是安全的
            # 对齐已跑通的 tube_grip 推理：action 是末端绝对位姿 + 夹爪开关，
            # 不是关节角。关节指令部署对不上 inference.py 的 EE 接口。
            action_space="ee_absolute_gripper",
        )
        _log(f"   → {res.get('out')}  ({res.get('frames')} 帧, "
             f"{res.get('episodes', 1)} episode)")
        return res
    except Exception as exc:  # noqa: BLE001
        _log(f"   WARN 导出 lerobot 失败: {exc}")
        _log("   （frames 会保留，不删 —— 可事后手动导出）")
        return {"error": str(exc)}


def prune_frames(
    session_dir: str,
    keep: int = 8,
    keep_meta: bool = True,
) -> dict:
    """删掉 frames/ 里的大部分原始帧，只留少量用于数据质监。

    **前置条件**：必须已经导出 lerobot 或 pack 过 —— 否则数据就丢了。
    调用方（auto_collect）会保证这个顺序。

    为什么留几帧而不是全删：
      - 出问题时（图像异常、相机位置错、深度坏）需要能回看
      - ``shots`` 命令（相机联络图）靠它做人工抽检
      - 8 帧（均匀分布）约 40MB，相比 4~5G 可忽略

    保留策略：**均匀采样**而不是取前 N 帧 —— 前后帧能看出动作进展，
    只留开头的话看不到任务中段/结尾的状态。

    Args:
        keep: 保留多少帧（0 = 全删）。
        keep_meta: 是否把「保留了哪几帧」记进 session.json，
            便于事后知道帧不连续（避免误当作完整序列处理）。

    Returns:
        ``{"deleted": N, "kept": [帧号...], "freed_mb": M}``
    """
    sess = Path(session_dir).expanduser()
    frames_dir = sess / "frames"
    if not frames_dir.is_dir():
        return {"deleted": 0, "kept": [], "freed_mb": 0}

    all_frames = sorted(frames_dir.glob("frame_*.npz"))
    n = len(all_frames)
    if n == 0:
        return {"deleted": 0, "kept": [], "freed_mb": 0}

    keep = max(0, int(keep))
    if keep >= n:
        return {"deleted": 0, "kept": list(range(n)), "freed_mb": 0}

    # 均匀采样：n 帧里挑 keep 个，等间距（含首末帧）
    if keep == 0:
        keep_idx: list[int] = []
    elif keep == 1:
        keep_idx = [0]
    else:
        keep_idx = sorted({
            round(i * (n - 1) / (keep - 1)) for i in range(keep)
        })
    keep_set = set(keep_idx)

    freed = 0
    deleted = 0
    for i, f in enumerate(all_frames):
        if i in keep_set:
            continue
        try:
            freed += f.stat().st_size
            f.unlink()
            deleted += 1
        except OSError as exc:
            _log(f"   WARN 删除 {f.name} 失败: {exc}")

    if keep_meta and deleted:
        try:
            sj = sess / "session.json"
            meta = json.loads(sj.read_text(encoding="utf-8")) if sj.is_file() else {}
            meta["frames_pruned"] = {
                "kept_indices": keep_idx,
                "kept_count": len(keep_idx),
                "deleted_count": deleted,
                "note": ("原始帧已按均匀采样裁剪，仅保留少量用于质监。"
                         "完整数据见同目录 episode.npz / episode_trimmed.npz，"
                         "以及 ~/lerobot_datasets/ 下的导出数据集。"
                         "**frames/ 已不连续，不要再当作完整序列处理**"),
            }
            sj.write_text(json.dumps(meta, indent=2, ensure_ascii=False),
                          encoding="utf-8")
        except Exception as exc:  # noqa: BLE001
            _log(f"   WARN 写 session.json 失败: {exc}")

    return {
        "deleted": deleted,
        "kept": keep_idx,
        "freed_mb": round(freed / (1024 ** 2), 1),
    }


def export_rounds_as_one_dataset(
    session_dirs: list[str],
    dataset_name: str,
    *,
    robot_type: str = "",
    prune_keep: int = 8,
    allow_mismatch: bool = False,
) -> dict:
    """把**多轮**导成**一个** LeRobot 数据集（多个 episode），成功后清理 frames。

    为什么用这个而不是逐轮导出
    -------------------------
    视频编码是导出的绝对大头（约 45s / 800 帧，占 95%），且按 episode 逐个
    编码、无法合并。所以逐轮导出 vs 批量导出，**时间上几乎没差别**（只差
    每次进程启动 + import lerobot 的 ~40ms）。

    真正的区别在**产物结构**：

    - 逐轮导出 → 20 个独立数据集（`run_046/`、`run_047/`…）
      训练时要循环加载 20 个再合并，且 normalization stats 各算各的。
    - 批量导出 → 1 个数据集、20 个 episode（`sinopec_test_tube_20eps/`）
      直接 ``LeRobotDataset(root=...)`` 就是完整训练集。

    另外批量导出会做**参数一致性守门**（相机分辨率/every 等），
    逐轮导出发现不了这类不一致。

    磁盘代价：攒着不删期间占用大（约 5G/轮）。所以默认保留 keep=8 帧
    是在导出**成功后**才执行的 —— 失败则原样保留，不会丢数据。

    Args:
        dataset_name: 数据集目录名（放在 ~/lerobot_datasets/ 下）。
        allow_mismatch: 参数不一致时是否强行合并（默认否，会报错并保留 frames）。

    Returns:
        ``{"ok", "out", "episodes", "frames", "pruned": [...]}``
    """
    from fasim.replicator.sdg import capture_profile as cp
    from fasim.replicator.sdg import export_lerobot as ex

    sessions = [Path(s).expanduser() for s in session_dirs]
    sessions = [s for s in sessions if (s / "frames").is_dir()]
    if not sessions:
        return {"ok": False, "error": "没有可导出的会话（frames 都不存在）"}

    out = Path.home() / "lerobot_datasets" / dataset_name

    # 参数一致性守门：不一致时无法合成一个合法数据集
    if len(sessions) > 1:
        try:
            rep = cp.compare(sessions)
            if not rep.ok:
                msg = ("采集参数不一致，无法合并成一个数据集。\n"
                       "  常见原因：某轮改了相机分辨率 / every / 相机数量")
                _log(f"   WARN {msg}")
                if not allow_mismatch:
                    return {
                        "ok": False,
                        "error": msg + "（frames 已保留，可逐轮单独导出）",
                        "mismatch": True,
                    }
        except Exception as exc:  # noqa: BLE001
            _log(f"   WARN 参数一致性检查失败（继续）: {exc}")

    _log(f"   合并导出 {len(sessions)} 轮 → {out}")
    _log(f"   （视频编码约 {len(sessions) * 45}s，请稍候）")
    try:
        res = ex.export_session(
            sessions,
            out,
            robot_type=robot_type or None,
            overwrite=True,
            # 与逐轮导出同一语义：ee_absolute_gripper（16 维），对齐 tube_grip。
            action_space="ee_absolute_gripper",
        )
    except Exception as exc:  # noqa: BLE001
        _log(f"   ERROR 导出失败: {exc}")
        _log("   frames 全部保留（可事后重试）")
        return {"ok": False, "error": str(exc)}

    _log(f"   → {res.get('out')}  ({res.get('frames')} 帧, "
         f"{res.get('episodes')} episode)")

    # 导出成功才清理 frames
    pruned = []
    if prune_keep >= 0:
        total_mb = 0.0
        for s in sessions:
            pr = prune_frames(str(s), keep=prune_keep)
            pruned.append({"session": s.name, **pr})
            total_mb += pr["freed_mb"]
        _log(f"   清理 {len(sessions)} 个包的原始帧，共释放 "
             f"{total_mb / 1024:.1f}GB（每包保留 {prune_keep} 帧）")

    return {
        "ok": True,
        "out": res.get("out"),
        "episodes": res.get("episodes"),
        "frames": res.get("frames"),
        "pruned": pruned,
    }


def trim_and_pack(session_dir: str, head_sec: float = 2.0,
                  tail_sec: float = 1.5, do_trim: bool = True,
                  do_pack: bool = True, do_export: bool = False,
                  prune_keep: int = -1, robot_type: str = "",
                  export_root: str = "") -> dict:
    """裁待机段 → 打包 → 导出 lerobot → 删原始 frames。

    **顺序不能改**：export 依赖 frames 里的 RGB，必须在删之前跑完。

    Args:
        do_export: 是否导出 lerobot 格式（需要 lerobot python/venv-train）。
        prune_keep: 导出成功后保留多少帧（-1 = 不删；0 = 全删；
            默认 8 = 均匀保留 8 帧用于质监）。**只有 export 成功才会删** ——
            否则 frames 是唯一图像来源，删了就导不出来了。
    """
    from fasim.replicator.sdg import pack, trim

    out = {"session": session_dir}

    if do_trim:
        _log("   裁剪：留开头 home + 截到回 home")
        rep = trim.trim_session(
            session_dir,
            mode="whole",
            head_sec=head_sec,
            tail_sec=tail_sec,
            write=True,
        )
        seg = (rep.get("segments") or [{}])[0]
        out["trimmed"] = rep.get("written", "")
        out["trim_report"] = {
            "frames_in": rep.get("frames_total"),
            "frames_out": rep.get("kept_frames"),
            "duration_sec": seg.get("duration_sec"),
            "start_frame": seg.get("start_frame"),
            "end_frame": seg.get("end_frame"),
        }
        _log(f"   {rep.get('kept_frames')}/{rep.get('frames_total')} 帧"
             f"（{seg.get('duration_sec')}s）")

    if do_pack:
        _log("   打包 episode.npz")
        try:
            p = pack.pack_episode(session_dir)
            out["episode"] = str(p)
            _log(f"   → {p}")
        except Exception as exc:  # noqa: BLE001
            _log(f"   WARN pack 失败（数据本身没问题）: {exc}")
            out["pack_error"] = str(exc)

    # ---- 导出 lerobot（可选：单轮模式）----
    #
    # batch_later 模式下 do_export=False（run_batch 跑完后统一导出），
    # 这里只 trim+pack，frames 全部保留。
    export_ok = False
    if do_export:
        _log("   导出 LeRobot 数据集")
        try:
            res = export_to_lerobot(
                session_dir, out_root=export_root, robot_type=robot_type
            )
            out["lerobot"] = res.get("out")
            export_ok = not res.get("error")
            if not export_ok:
                out["export_error"] = res.get("error")
        except Exception as exc:  # noqa: BLE001
            _log(f"   WARN 导出异常: {exc}")
            out["export_error"] = str(exc)
    else:
        # 没要求导出：pack 已经把状态/末端/相机参数存进 episode.npz，
        # 但**图像只在 frames 里**。所以这种情况下不删 frames。
        export_ok = False

    # ---- 删原始帧（只保留少量质监用）----
    #
    # **只有以下两种情况可以删**：
    #   1) 本次导出 lerobot 成功（图像已进 mp4）
    #   2) 目录里已有 lerobot 数据集（之前导过）
    # 否则 frames 是唯一的图像来源，删了就再也导不出训练格式。
    if prune_keep >= 0:
        can_prune = export_ok
        if not can_prune and not do_export:
            # 没要求导出 —— 检查是否已有现成数据集（比如手工导过）
            existing = Path.home() / "lerobot_datasets" / Path(session_dir).name
            can_prune = (existing / "meta" / "info.json").is_file()
            if can_prune:
                _log(f"   发现已有数据集 {existing.name} —— 可安全清理 frames")
        if can_prune:
            _log(f"   清理原始帧（保留 {prune_keep} 帧用于质监）")
            pr = prune_frames(session_dir, keep=prune_keep)
            out["frames_pruned"] = pr
            if pr["deleted"]:
                _log(f"   删除 {pr['deleted']} 帧，释放 {pr['freed_mb']}MB，"
                     f"保留帧号 {pr['kept']}")
        else:
            _log("   WARN 没有可用的 lerobot 数据集 —— 保留全部 frames"
                 "（否则再也导不出训练格式）")

    return out


def run_round(
    *,
    robot_key: str,
    task_key: str,
    scene: str = "default",
    every: int = 10,
    camera: str = "all",
    head_sec: float = 2.0,
    tail_sec: float = 1.5,
    do_trim: bool = True,
    do_pack: bool = True,
    out_dir: str = "",
    skip_randomize: bool = False,
    do_export: bool = False,
    prune_keep: int = -1,
    robot_type: str = "",
) -> dict:
    """跑**一轮**完整流程。返回该轮的汇总 dict。

    任一步失败立即返回（``ok=False`` 且 ``failed_at`` 指出卡在哪），
    **不删数据** —— 半途失败时最需要看现场。
    """
    from fasim.replicator.sdg import runtime

    t0 = time.monotonic()
    summary: dict = {
        "ok": False,
        "robot_key": robot_key,
        "task_key": task_key,
        "failed_at": "",
    }

    # ---- 0. 任务上下文 ----
    try:
        ctx = _resolve_task_ctx(robot_key, task_key)
    except Exception as exc:  # noqa: BLE001
        _log(f"ERROR 读任务配置失败: {exc}")
        summary["failed_at"] = "load_config"
        summary["error"] = str(exc)
        return summary

    # 用 run_NNN 序号命名（与本项目历史数据一致）。
    # default_session_dir() 是时间戳风格，混用会导致排序/批量脚本出问题。
    target_out = str(out_dir) if out_dir else str(
        runtime.next_run_dir(robot_key, task_key)
    )
    summary["out_dir"] = target_out

    # ---- 1. 快照 ----
    if not skip_randomize:
        _log("[1/5] 检查初始位姿快照")
        if not ensure_snapshot(robot_key, task_key, ctx):
            summary["failed_at"] = "snapshot"
            return summary

        # ---- 2. 随机化 ----
        _log("[2/5] 随机化物体")
        ok, _ = randomize_once(robot_key, task_key, ctx)
        if not ok:
            summary["failed_at"] = "randomize"
            return summary
    else:
        _log("[1-2/5] 跳过快照与随机化")

    # ---- 3. 开始采集 ----
    _log("[3/5] 开始采集")
    ok, actual_out = start_capture(
        robot_key, task_key, target_out, every=every, camera=camera
    )
    if not ok:
        summary["failed_at"] = "capture_start"
        return summary
    summary["out_dir"] = actual_out

    # ---- 4. 跑编排 ----
    _log("[4/5] 跑编排")
    rc = run_orchestration(robot_key, task_key, scene=scene)
    summary["motion_rc"] = rc
    if rc != 0:
        _log(f"   WARN 编排退出码 {rc}（可能超时/中断，但仍会收尾保存数据）")

    # ---- 5. 结束采集 ----
    _log("[5/5] 结束采集")
    ok, stop_resp = stop_capture()
    if not ok:
        summary["failed_at"] = "capture_stop"
        summary["error"] = stop_resp.get("error", "")
        return summary

    summary["frames"] = stop_resp.get("frames_saved", 0)
    summary["stop_reason"] = stop_resp.get("stop_reason", "")
    summary["done"] = stop_resp.get("done")
    # RTF：墙钟每秒物理步数。**逐轮下降是长跑的主要问题**（实测 5 轮 -33%），
    # 记下来便于在汇总/提醒里观察劣化趋势。
    summary["rtf_hz"] = stop_resp.get("physics_hz_wall")
    dur = stop_resp.get("duration_sec") or 0
    if dur > 0 and summary["frames"]:
        summary["fps"] = round(summary["frames"] / dur, 1)

    # ---- 6. 裁剪 + 打包 ----
    if summary["frames"] > 0:
        _log("后处理：裁剪 + 打包")
        try:
            products = trim_and_pack(
                actual_out,
                head_sec=head_sec,
                tail_sec=tail_sec,
                do_trim=do_trim,
                do_pack=do_pack,
                do_export=do_export,
                prune_keep=prune_keep,
                robot_type=robot_type or robot_key,
            )
            summary["products"] = products
        except Exception as exc:  # noqa: BLE001
            _log(f"   WARN 后处理失败（原始帧仍在 {actual_out}）: {exc}")
            summary["postprocess_error"] = str(exc)
    else:
        _log("WARN 一帧都没采到 —— 检查 Isaac 的 timeline 是否 PLAY")

    summary["ok"] = summary["frames"] > 0
    summary["elapsed_sec"] = round(time.monotonic() - t0, 1)
    return summary


def run_batch(
    *,
    robot_key: str,
    task_key: str,
    rounds: int = 1,
    scene: str = "default",
    every: int = 10,
    camera: str = "all",
    head_sec: float = 2.0,
    tail_sec: float = 1.5,
    do_trim: bool = True,
    do_pack: bool = True,
    stop_on_fail: bool = True,
    skip_randomize: bool = False,
    retries: int = 1,
    restart_hint_every: int = 5,
    do_export: bool = False,
    prune_keep: int = 8,
    batch_export: bool = True,
) -> list[dict]:
    """跑 N 轮。每轮独立（各自随机化 + 独立目录）。

    Args:
        retries: 某轮失败后重试几次（默认 1）。长跑（几十轮）时建议保留 ——
            偶发的 socket 超时、编排进程异常退出都会让单轮失败，但下一轮
            通常就好了。重试会**用新的 run_NNN 目录**（失败的目录保留，
            便于排查，也能人工删）。
        stop_on_fail: 重试用尽后是否中止整个批次。
        restart_hint_every: 每跑这么多轮提醒一次「建议重启 Isaac」。
            **实测 RTF 会逐轮下降**（138Hz → 92Hz / 5 轮，降幅约 33%），
            重启后恢复。原因未能完全定位（已排除内存、annotator 累积），
            怀疑是 render product 在 Replicator 内部注册表的累积
            （代码特意不 destroy 以避免破坏 ROS 图，见 live_capture 注释）。
            设为 0 可关闭提醒。
    """
    results = []
    total = max(1, int(rounds))
    max_try = max(0, int(retries))

    for i in range(total):
        _log("=" * 62)
        _log(f"第 {i + 1}/{total} 轮")
        _log("=" * 62)

        r: dict = {}
        for attempt in range(max_try + 1):
            if attempt:
                _log(f"  第 {attempt} 次重试（上次失败于 "
                     f"{r.get('failed_at')}）")
            r = run_round(
                robot_key=robot_key,
                task_key=task_key,
                scene=scene,
                every=every,
                camera=camera,
                head_sec=head_sec,
                tail_sec=tail_sec,
                do_trim=do_trim,
                do_pack=do_pack,
                skip_randomize=skip_randomize,
                # batch_export 模式下单轮不导出（循环结束后统一导），
                # 这样能拿到「一个数据集多 episode」而不是 N 个独立数据集。
                do_export=(do_export and not batch_export),
                prune_keep=(-1 if batch_export else prune_keep),
                robot_type=robot_key,
            )
            if r.get("ok"):
                break

        r["attempts"] = attempt + 1
        results.append(r)
        tag = "OK" if r.get("ok") else f"失败@{r.get('failed_at')}"
        _log(f"第 {i + 1} 轮结束: {tag}，用时 {r.get('elapsed_sec', 0)}s"
             + (f"（重试 {attempt} 次）" if attempt else ""))
        if not r.get("ok") and stop_on_fail:
            _log("重试用尽仍失败，按 stop_on_fail 中止后续轮次")
            break

        # 每跑 N 轮提醒重启 Isaac（FPS 逐轮下降，重启可恢复）
        done = i + 1
        if (restart_hint_every > 0 and done % restart_hint_every == 0
                and done < total):
            rtf = r.get("rtf_hz")
            _log("")
            _log("─" * 62)
            _log(f"已跑 {done} 轮" + (f"，本轮 RTF≈{rtf:.0f}Hz" if rtf else ""))
            _log("建议现在重启 Isaac —— 实测 RTF 逐轮下降（5 轮约 -33%）")
            _log("  1) 关掉 Isaac 窗口，重新 ./run.sh 并 Play 到待命状态")
            _log(f"  2) 再跑剩下的 {total - done} 轮:")
            _log(f"     sdg.sh auto --robot {robot_key} --task {task_key} "
                 f"--rounds {total - done}")
            _log("  数据目录按 run_NNN 递增，不会覆盖已有包")
            _log("─" * 62)
            _log("")

    # 汇总
    # ---- 批量导出：把本轮所有成功的包合并成一个 LeRobot 数据集 ----
    #
    # 为什么放在循环**之后**而不是每轮导：
    #   - 视频编码是导出主要成本（~45s/包，占 95%），按 episode 编码、
    #     无法合并 —— 所以时间上「逐轮导」和「批量导」几乎一样。
    #   - 但产物结构差别大：批量导 = 1 个数据集 N 个 episode（训练直接用），
    #     逐轮导 = N 个独立数据集（训练时要循环加载再合并）。
    #   - 批量导还能做参数一致性守门（相机分辨率/every 不一致会拦下）。
    #
    # 代价：攒着期间每轮占约 5G。导出成功后立刻清理（每包留 prune_keep 帧）。
    # --no-export 只关 LeRobot。每轮的 trim / pack（episode.npz）不受影响。
    if do_export and batch_export and prune_keep >= 0:
        ok_sessions = [r["out_dir"] for r in results
                       if r.get("ok") and r.get("out_dir")
                       and Path(r["out_dir"], "frames").is_dir()]
        if ok_sessions:
            _log("=" * 62)
            _log(f"批量导出：{len(ok_sessions)} 轮 → 一个 LeRobot 数据集")
            _log("=" * 62)
            ds_name = f"{task_key}_{len(ok_sessions)}eps"
            be = export_rounds_as_one_dataset(
                ok_sessions,
                ds_name,
                robot_type=robot_key,
                prune_keep=prune_keep,
            )
            for r in results:
                r["batch_export"] = be if r.get("ok") else None
            if not be.get("ok"):
                _log(f"WARN 批量导出未完成: {be.get('error')}")
                _log("     frames 已保留，可稍后手动重试：")
                _log(f"     sdg.sh export <任务目录> --out ~/lerobot_datasets/{ds_name}")
        else:
            _log("没有成功的会话可导出")

    _log("=" * 62)
    ok_n = sum(1 for r in results if r.get("ok"))
    _log(f"汇总: {ok_n}/{len(results)} 轮成功")
    for i, r in enumerate(results, 1):
        if r.get("ok"):
            tr = (r.get("products") or {}).get("trim_report") or {}
            rtf = r.get("rtf_hz")
            rtf_s = f" RTF={rtf:.0f}Hz" if rtf else ""
            _log(f"  {i}. {Path(r.get('out_dir', '')).name:12s} "
                 f"{r.get('frames')} 帧 → {tr.get('frames_out', '?')} 帧"
                 f" stop_reason={r.get('stop_reason')}{rtf_s}")
        else:
            _log(f"  {i}. 失败于 {r.get('failed_at')}: {r.get('error', '')}")

    # RTF 趋势 —— 长跑最需要盯的指标（逐轮下降说明 Isaac 状态在劣化）
    rtfs = [r["rtf_hz"] for r in results if r.get("rtf_hz")]
    if len(rtfs) >= 2:
        drop = rtfs[0] - rtfs[-1]
        pct = drop / rtfs[0] * 100 if rtfs[0] else 0
        _log(f"RTF 趋势: {rtfs[0]:.0f}Hz → {rtfs[-1]:.0f}Hz "
             f"（{drop:+.0f}Hz / {pct:+.1f}%）")
        if pct > 20:
            _log("⚠ 劣化超过 20%，建议重启 Isaac 后再继续采集")
    return results
