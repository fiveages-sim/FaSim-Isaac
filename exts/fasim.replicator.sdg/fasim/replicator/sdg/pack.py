"""层2.5 采集 → episode npz → 层2 回放，三步打通。

为什么需要这个
==============
层2.5 的产物是**每帧一个 npz**（``frame_XXXXXX.npz``），层2 的输入是
**一个 episode npz**（``joint_pos`` (N,J) + ``t`` + 末端/物体位姿）。
两者格式不同，中间缺一步转换。

有了转换就能闭环自检：

::

    sdg.sh capture        采集（真实闭环跑编排）
        ↓  frames/frame_*.npz
    sdg.sh pack           转成 episode npz（本模块）
        ↓  episode.npz
    sdg.sh preview        用层2 的驱动回放一遍，肉眼确认关节都录到了

``preview`` 复用 ``renderer.render_episode`` —— 那是已经调通的物理重演
路径（锁底盘 + drive 驱动 + 夹爪开关量），能真实看出动作对不对。

关键：不做任何"修正"
====================
转换只做**格式搬运**，不改数值、不插值、不填补。这样 ``preview`` 出来的
动作如果和编排时一致，就证明采集本身是忠实的；不一致则说明采集有问题。
"""

from __future__ import annotations

import json
from pathlib import Path
from typing import Any

import numpy as np

SCHEMA_VERSION = 2


def _log(msg: str) -> None:
    print(f"[pack] {msg}", flush=True)


def load_session(session_dir: str | Path) -> tuple[list[dict], dict, Path]:
    """读一个采集会话目录 → (帧元数据列表, session.json, 目录)。"""
    d = Path(session_dir).expanduser().resolve()
    if not d.is_dir():
        raise FileNotFoundError(d)
    frames_dir = d / "frames"
    if not frames_dir.is_dir():
        raise FileNotFoundError(f"没有 frames 目录: {frames_dir}")

    meta: dict = {}
    sj = d / "session.json"
    if sj.is_file():
        try:
            meta = json.loads(sj.read_text(encoding="utf-8"))
        except Exception as exc:  # noqa: BLE001
            _log(f"WARN 读 session.json 失败: {exc}")

    files = sorted(frames_dir.glob("frame_*.npz"))
    if not files:
        raise ValueError(f"没有帧文件: {frames_dir}")
    return [{"path": f} for f in files], meta, d


def pack_episode(
    session_dir: str | Path,
    out_path: str | Path | None = None,
    frame_dt: float | None = None,
) -> Path:
    """把采集会话转成层2 能读的 episode npz。

    Args:
        session_dir: ``sdg.sh capture`` 产出的会话目录。
        out_path: 输出 npz；留空 = ``<session_dir>/episode.npz``。
        frame_dt: 帧间隔（秒）。留空从 ``physics_dt × every`` 推算，
            取不到就用 0.05（20fps）。

    Returns:
        写出的 npz 路径。
    """
    entries, meta, d = load_session(session_dir)
    out = Path(out_path).expanduser() if out_path else d / "episode.npz"

    # --- 逐帧读取，按 key 归并 ---
    cols: dict[str, list[Any]] = {}
    joint_names: list[str] | None = None
    sim_steps: list[int] = []
    has_cam = False

    for i, ent in enumerate(entries):
        with np.load(ent["path"], allow_pickle=False) as z:
            keys = set(z.files)
            if i == 0:
                joint_names = (
                    [str(x) for x in z["joint_names"]] if "joint_names" in keys else None
                )
            sim_steps.append(int(z["sim_step"]) if "sim_step" in keys else i)

            # 相机：只记录"有哪些"，不搬图像（图像留在 frames/ 里）
            for k in keys:
                if k.endswith("_rgb") or k.endswith("_depth"):
                    has_cam = True

            # 关节
            if "joint_pos" in keys:
                q = np.asarray(z["joint_pos"], dtype=np.float64).reshape(-1)
                cols.setdefault("joint_pos", []).append(q)
            # 下发目标（回放用）。老会话没有这个字段，回退到实测值。
            if "joint_pos_cmd" in keys:
                qc = np.asarray(z["joint_pos_cmd"], dtype=np.float64).reshape(-1)
                cols.setdefault("joint_pos_cmd", []).append(qc)

            # 末端 + 物体：<side>_pos/_quat 与 obj/<label>_pos/_quat
            for k in keys:
                if k in ("joint_pos", "joint_vel", "joint_names", "joint_pos_cmd"):
                    continue
                if k.endswith("_pos") or k.endswith("_quat"):
                    cols.setdefault(k, []).append(
                        np.asarray(z[k], dtype=np.float64).reshape(-1)
                    )
                # Xform 对照值：同样搬，但名字带 _xform，绝不参与末端主值
                elif k.endswith("_pos_xform") or k.endswith("_quat_xform"):
                    cols.setdefault(k, []).append(
                        np.asarray(z[k], dtype=np.float64).reshape(-1)
                    )

    if "joint_pos" not in cols:
        raise ValueError(
            "会话里没有 joint_pos（采集时关了 record_joints？）。"
            "层2 回放必须有关节角。"
        )

    # --- 时间轴 ---
    #
    # 帧间隔 = 每帧物理步数 × **仿真**物理步长。
    #
    # 关键：``physics_dt`` 是**仿真时间**步长（配置的 5ms），不是墙钟耗时。
    # 本机墙钟约 6.5ms/步，那是渲染/计算开销导致 RTF<1，跟时间轴无关。
    #
    # 曾经用「墙钟 / 步数」反推，得到 5.876ms 并当成真值，commit 5c75702
    # 据此"修正"了 5ms 硬编码 —— 那是错的：按 5.876ms 算会让回放**慢** 15%，
    # 手到得晚。fix01 用 5ms 回放与编排完全一致（且插拔任务成功），这是
    # 直接反证。
    #
    # 所以顺序改成：
    #   1) 调用方显式给 frame_dt
    #   2) session.json 的 physics_dt（采集时 SimulationManager 读的**仿真**步长）
    #   3) 兜底 5ms（G1D 默认配置；明确提示用 --frame-dt 覆盖）
    # **不再用墙钟反推** —— 它量的是性能，不是时间轴。
    dt = frame_dt
    if dt is None:
        every = int(meta.get("every", 0) or 0)
        dsteps = np.diff(sim_steps) if len(sim_steps) >= 2 else np.array([])
        med = float(np.median(dsteps)) if dsteps.size else float(every or 10)

        phys_dt = float(meta.get("physics_dt", 0) or 0)
        src = "session.json physics_dt（仿真步长）"
        if phys_dt <= 0:
            phys_dt = 0.005
            src = "兜底 5ms（老会话没记 physics_dt）"
            _log("WARN session.json 没有 physics_dt，用默认 5ms。"
                 "若回放速度不对，用 --frame-dt 手动指定")
        dt = med * phys_dt
        _log(f"帧间隔推算: {med:g} 物理步 × {phys_dt*1000:.3f}ms "
             f"= {dt*1000:.2f}ms   来源: {src}")
        # 顺带提示 RTF —— 只影响采集合规性（够不够 30fps），不影响时间轴
        n_phys = float(meta.get("physics_steps", 0) or 0)
        dur = float(meta.get("duration_sec", 0) or 0)
        if n_phys > 0 and dur > 0:
            rtf = (n_phys * phys_dt) / dur
            if rtf < 0.95:
                _log(f"  注: 实时率 RTF≈{rtf:.2f}（墙钟 {dur/n_phys*1000:.2f}ms/步），"
                     "仿真比实时慢是正常的，不影响时间轴")
    dt = float(dt)
    n = len(cols["joint_pos"])
    t = np.arange(n, dtype=np.float64) * dt

    # --- 组装 episode ---
    joint_pos = np.stack(cols["joint_pos"]).astype(np.float64)
    # 下发目标：缺了就回退实测，并在报告里说清楚（这会多叠一次稳态误差）
    if "joint_pos_cmd" in cols and len(cols["joint_pos_cmd"]) == n:
        joint_pos_cmd = np.stack(cols["joint_pos_cmd"]).astype(np.float64)
    else:
        joint_pos_cmd = None

    payload: dict[str, Any] = {
        "schema_version": np.array(SCHEMA_VERSION),
        "t": t,
        "joint_pos": joint_pos,
        "joint_names": np.array(joint_names or [f"j{i}" for i in range(joint_pos.shape[1])]),
        "frame_id": np.array("world"),
        "meta": np.array(
            json.dumps(
                {
                    "kind": "live_capture",
                    "source_session": str(d),
                    "frames": n,
                    "frame_dt": dt,
                    "every": meta.get("every"),
                    "cameras": [c["name"] for c in meta.get("cameras", [])],
                    "has_camera_frames": has_cam,
                    "note": "由 sdg.sh pack 从 frames/frame_*.npz 转换；图像仍在 frames/",
                },
                ensure_ascii=False,
            )
        ),
    }

    # 末端位姿：Episode.load 需要 left_pos/left_quat/right_pos/right_quat 齐全
    for side, default in (("left", None), ("right", None)):
        p = cols.get(f"{side}_pos")
        q = cols.get(f"{side}_quat")
        if p and q and len(p) == n:
            payload[f"{side}_pos"] = np.stack(p).astype(np.float64)
            payload[f"{side}_quat"] = np.stack(q).astype(np.float64)
        else:
            # 缺失时填 NaN 占位，Episode.load 能过，但 preview 不看这个
            payload[f"{side}_pos"] = np.full((n, 3), np.nan)
            payload[f"{side}_quat"] = np.full((n, 4), np.nan)

    for k, v in cols.items():
        if k.startswith("obj/") and len(v) == n:
            payload[k] = np.stack(v).astype(np.float64)
        # Xform 对照值一并保留（诊断用，训练时要忽略 *_xform）
        elif (k.endswith("_pos_xform") or k.endswith("_quat_xform")) and len(v) == n:
            payload[k] = np.stack(v).astype(np.float64)

    # 下发目标关节角 —— 回放用它。
    if joint_pos_cmd is not None:
        payload["joint_pos_cmd"] = joint_pos_cmd
        payload["meta"] = np.array(
            json.dumps(
                {**json.loads(str(payload["meta"])), "has_joint_cmd": True},
                ensure_ascii=False,
            )
        )

    out.parent.mkdir(parents=True, exist_ok=True)
    np.savez_compressed(out, **payload)

    _log(f"已写出 {out}")
    _log(f"  {n} 帧 × {joint_pos.shape[1]} 关节，dt={dt*1000:.2f}ms")

    # 稳态误差报告 —— 这个数字直接解释了"回放偏下几厘米"。
    if joint_pos_cmd is not None:
        e = np.abs(joint_pos - joint_pos_cmd)
        emax = e.max(axis=0)
        k = int(np.argmax(emax))
        nm = (joint_names or [f"j{i}" for i in range(e.shape[1])])[k]
        _log(f"  drive 稳态误差: mean={e.mean()*1000:.2f} max={emax[k]*1000:.1f} mrad"
             f"（最大 @{nm}）")
        _log("  → 回放会用 joint_pos_cmd（下发目标），避免误差叠加两次")
    else:
        _log("  WARN 没有 joint_pos_cmd（下发目标）—— 回放将用实测值，"
             "末端会比编排多偏一次稳态误差 ≈ 几厘米")
        _log("       重新采集即可获得：sdg.sh capture --what joint,ee,object")
    ee = [
        k for k in payload
        if k.endswith("_pos") and not k.startswith("obj/") and k != "joint_pos"
        and np.isfinite(np.asarray(payload[k])).any()
    ]
    ob = [k for k in payload if k.startswith("obj/") and k.endswith("_pos")]
    _log(f"  末端: {ee or '无'}   物体: {[k[len('obj/'):-4] for k in ob] or '无'}")

    # 末端是真动态还是死值 —— 直接给结论，不用人读 npz
    for side in ("left", "right"):
        pv = payload.get(f"{side}_pos")
        if pv is None or not np.isfinite(np.asarray(pv)).any():
            continue
        pv = np.asarray(pv)
        travel = float(np.linalg.norm(pv - pv[0], axis=1).max() * 1000)
        tag = "动态 ✅" if travel >= 10.0 else "静态 ❌"
        line = f"  末端 {side}: {tag} travel={travel:.1f}mm"
        xv = payload.get(f"{side}_pos_xform")
        if xv is not None:
            xv = np.asarray(xv)
            xt = float(np.linalg.norm(xv - xv[0], axis=1).max() * 1000)
            d = np.linalg.norm(pv - xv, axis=1) * 1000
            line += f"  |  Xform 对照 travel={xt:.1f}mm 差值 mean={d.mean():.1f}mm"
        _log(line)
    if any(k.endswith("_pos_xform") for k in payload):
        _log("  注: *_xform 是 Xform 直读的静态对照值，训练时必须忽略")
    if has_cam:
        _log(f"  相机图像仍在 {d / 'frames'}（未并入 npz）")
    # 缺末端/物体时明确告警（否则 pack 只说「无」，看不出是采集没采还是
    # 转换丢了）
    if not ee:
        _log("  WARN 没有末端位姿 —— 采集时要选 ee：sdg.sh capture --what joint,ee")
    if not ob:
        _log("  WARN 没有物体位姿 —— 采集时要选 object："
             "sdg.sh capture --what joint,ee,object")
        cap = meta.get("objects_captured")
        if cap is not None:
            _log(f"       但 session.json 记录采集时有 {cap} —— 说明是写帧时丢的，"
                 "检查 Isaac 终端有没有「WARN 物体 ... 取值失败」")
    stamp = meta.get("code_stamp")
    if stamp:
        _log(f"  采集时的代码版本: {stamp}")
    return out


def summarize(session_dir: str | Path) -> dict:
    """不转换，只报告会话里有什么 —— ``sdg.sh info`` 用。"""
    entries, meta, d = load_session(session_dir)
    with np.load(entries[0]["path"], allow_pickle=False) as z:
        keys = list(z.files)
        jn = [str(x) for x in z["joint_names"]] if "joint_names" in z.files else []
    info = {
        "session": str(d),
        "frames": len(entries),
        "every": meta.get("every"),
        "physics_steps": meta.get("physics_steps"),
        "duration_sec": meta.get("duration_sec"),
        "errors": meta.get("capture_errors"),
        "cameras": [c["name"] for c in meta.get("cameras", [])],
        "first_frame_keys": keys,
        "joint_count": len(jn),
        "size_mb": round(
            sum(f["path"].stat().st_size for f in entries) / 1e6, 1
        ),
    }
    return info
