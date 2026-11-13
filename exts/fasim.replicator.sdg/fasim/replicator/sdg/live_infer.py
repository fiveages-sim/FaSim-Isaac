"""在**正在运行的 Isaac GUI 实例**里做闭环推理（L3）—— 机器人实时动起来。

与 ``live_replay`` 的区别
========================
| | ``live_replay`` | 本模块 |
|---|---|---|
| 动作来源 | ``episode.npz`` 里预存的 ``joint_pos_cmd`` | **策略实时计算** |
| 观测 | 不需要（照着演） | **每帧实时读相机 + 关节** |
| 能否应对偏差 | 不能（开环播放） | 能（闭环：看到什么做什么） |

所以本模块是「真正跑策略」：每次 ``app.update()`` 里
1. 从 Replicator annotator 读 3 路 RGB（当前画面）
2. 读当前关节角
3. 组装 batch → ``policy.select_action``
4. 反归一化 → 32 维关节目标角
5. ``set_dof_position_targets`` 下发
6. 物理推进一步，回到 1

为什么不在回调里跑 for 循环
==========================
和 ``live_replay`` 一样：在 socket 回调 / Script Editor 里调 ``app.update()``
会**重入 Kit 主循环**，画面卡死或崩溃。所以这里只「装载 + 注册回调」，
真正的逐步驱动在 ``_on_update`` 里。

用法
====
在 Isaac 的 Script Editor 里（场景已加载、已 Play）::

    import sys; sys.path.insert(0, "/home/fiveages/FaSim-Isaac/exts/fasim.replicator.sdg")
    from fasim.replicator.sdg import live_infer
    live_infer.start(
        checkpoint="/home/fiveages/act_runs/tube25/checkpoints/020000/pretrained_model",
        robot_key="unitree_g1d",
        task_key="sinopec_test_tube",
        camera="all",
        max_frames=600,
    )

    live_infer.status()    # 看进度
    live_infer.stop()      # 停

或经 CLI（``sdg.sh policy live ...``，见 cli.py）。
"""

from __future__ import annotations

from pathlib import Path
from typing import Any

import numpy as np

_STATE: dict[str, Any] = {
    "playing": False,
    "sub": None,
    "policy": None,
    "pre": None,
    "post": None,
    "cams": [],
    "art": None,
    "dof_names": [],
    "target_idx": None,
    "frame": 0,
    "max_frames": 600,
    "errors": 0,
    "device": "cpu",
    "grip_specs": [],
}


def _log(msg: str) -> None:
    print(f"[infer] {msg}", flush=True)


def is_playing() -> bool:
    return bool(_STATE["playing"])


def status() -> dict:
    return {
        "playing": bool(_STATE["playing"]),
        "frame": int(_STATE["frame"]),
        "max_frames": int(_STATE["max_frames"]),
        "errors": int(_STATE["errors"]),
        "device": _STATE["device"],
        "cameras": [c["name"] for c in _STATE["cams"]],
    }


def stop() -> dict:
    """停止推理并释放相机。"""
    if _STATE["sub"] is not None:
        try:
            _STATE["sub"].unsubscribe()
        except Exception:  # noqa: BLE001
            pass
    # 只 detach annotator，不 destroy render product（见 live_capture 的说明：
    # destroy 会破坏 Replicator 全局状态，导致 ROS 图的 writer 报 invalid null prim）
    for cam in _STATE["cams"]:
        for key in ("rgb", "depth"):
            ann = cam.get(key)
            if ann is None:
                continue
            try:
                rp = cam.get("rp")
                if rp is not None:
                    ann.detach(rp)
            except Exception:  # noqa: BLE001
                pass
    _STATE.update(playing=False, sub=None, cams=[], frame=0)
    _log("已停止")
    return {"ok": True, "frame": _STATE["frame"]}


def _load_policy(checkpoint: str):
    """加载 ACTPolicy 与 pre/post processor（归一化）。

    **归一化不能省**：lerobot 0.5.x 的 ``select_action`` 只做前向，
    归一化在 ``PolicyProcessorPipeline`` 里（训练时保存到 checkpoint）。
    漏掉后预测值量级会差上千倍、大量饱和到 ±1。
    """
    from lerobot.policies.act.modeling_act import ACTPolicy

    ckpt = Path(checkpoint).expanduser()
    policy = ACTPolicy.from_pretrained(str(ckpt))
    policy.eval()

    pre = post = None
    try:
        from lerobot.processor.pipeline import PolicyProcessorPipeline

        for fname, tag in (("policy_preprocessor.json", "pre"),
                           ("policy_postprocessor.json", "post")):
            if (ckpt / fname).is_file():
                pipe = PolicyProcessorPipeline.from_pretrained(
                    pretrained_model_name_or_path=str(ckpt),
                    config_filename=fname,
                    to_transition=None,
                )
                if tag == "pre":
                    pre = pipe
                else:
                    post = pipe
    except Exception as exc:  # noqa: BLE001
        _log(f"WARN processor 加载失败（预测值不可信）: {exc}")
    return policy, pre, post


def start(
    checkpoint: str,
    robot_key: str = "",
    task_key: str = "",
    camera: str = "all",
    width: int | None = None,
    height: int | None = None,
    max_frames: int = 600,
    every_n_updates: int = 1,
    warmup_frames: int = 10,
) -> dict:
    """装载策略并注册 update 回调（非阻塞，立刻返回）。

    Args:
        checkpoint: ``pretrained_model`` 目录（含 model.safetensors +
            config.json + policy_{pre,post}processor.json）。
        robot_key / task_key: 用于读 configs（相机 prim 路径、关节映射）。
        camera: 相机名 / "all" / ""（不采图 —— 但 ACT 需要图像，会失败）。
        max_frames: 最多推理多少帧后自动停（防跑飞）。
        every_n_updates: 每 N 次 update 推理一次（降频，缓解算力竞争）。
        warmup_frames: 开头跳过几帧（等渲染/物理稳定）。
    """
    import omni.kit.app
    import omni.usd

    if _STATE["playing"]:
        return {"ok": False, "error": "已在推理中，先 stop_policy()"}

    st = omni.usd.get_context().get_stage()
    if st is None:
        return {"ok": False, "error": "当前没有打开的 stage —— 先加载场景"}

    from .config_io import load_robot_config, load_task_config

    if not str(robot_key or "").strip() or not str(task_key or "").strip():
        return {"ok": False, "error": "必须给 robot_key 和 task_key"}

    try:
        robot = load_robot_config(robot_key)
        task = load_task_config(robot_key, task_key)
    except Exception as exc:  # noqa: BLE001
        return {"ok": False, "error": f"读配置失败: {exc}"}

    if not st.GetPrimAtPath(robot.articulation_prim):
        return {"ok": False,
                "error": f"当前 stage 没有 {robot.articulation_prim} —— 先加载场景"}

    # ---- 策略 ----
    try:
        policy, pre, post = _load_policy(checkpoint)
    except Exception as exc:  # noqa: BLE001
        return {"ok": False, "error": f"加载 checkpoint 失败: {exc}"}
    import torch

    dev = next(policy.parameters()).device
    _log(f"策略已加载 device={dev}  pre={'有' if pre else '无'} post={'有' if post else '无'}")

    # ---- 关节 ----
    from isaacsim.core.experimental.prims import Articulation

    art = Articulation(robot.articulation_prim)
    dof_names = list(art.dof_names)
    _log(f"关节: {len(dof_names)} 个 DOF")

    # ---- 相机 ----
    import omni.replicator.core as rep

    all_specs = list(getattr(task, "cameras", None) or [])
    if not all_specs:
        return {"ok": False, "error": "任务配置里没有相机（ACT 必须有图像）"}
    sel = str(camera or "all").strip()
    if sel.lower() == "all":
        specs = all_specs
    else:
        want = {c.strip() for c in sel.split(",") if c.strip()}
        specs = [s for s in all_specs if s.name in want]
        if not specs:
            return {"ok": False,
                    "error": f"找不到相机 {want}，可选 {[s.name for s in all_specs]}"}
    cache = _STATE.setdefault("_rp_cache", {})
    cams: list[dict] = []
    for spec in specs:
        w = int(width or (spec.resolution[0] if spec.resolution else 640))
        h = int(height or (spec.resolution[1] if spec.resolution else 480))
        key = f"{spec.name}@{w}x{h}@{spec.prim_path}"
        rp = cache.get(key)
        if rp is None:
            try:
                rp = rep.create.render_product(spec.prim_path, (w, h))
                cache[key] = rp
            except Exception as exc:  # noqa: BLE001
                _log(f"WARN 相机 {spec.name} 创建失败: {exc}")
                continue
        rgb = rep.annotators.get("rgb")
        rgb.attach(rp)
        cams.append({"name": spec.name, "rp": rp, "rgb": rgb, "size": (w, h)})
    if not cams:
        return {"ok": False, "error": "所有相机都创建失败"}
    _log(f"相机: {[c['name'] for c in cams]}")

    _STATE.update(
        playing=True, policy=policy, pre=pre, post=post, cams=cams,
        art=art, dof_names=dof_names, frame=0, max_frames=int(max_frames),
        errors=0, device=str(dev), _every=max(1, int(every_n_updates)),
        _updates=0, _warmup=int(warmup_frames), sub=None,
    )

    def _on_update(_event: Any = None) -> None:
        """每次 app.update() 调一次 —— 必须快速返回，绝不能阻塞。"""
        if not _STATE["playing"]:
            return
        _STATE["_updates"] += 1
        if _STATE["_updates"] % _STATE["_every"] != 0:
            return
        if _STATE["_updates"] <= _STATE["_warmup"]:
            return

        try:
            import torch

            # 1) 关节角
            q = art.get_dof_positions()
            q = np.asarray(q, dtype=np.float32).reshape(-1)

            # 2) 图像
            batch: dict[str, Any] = {
                "observation.state": torch.from_numpy(q).unsqueeze(0).to(dev),
            }
            for cam in _STATE["cams"]:
                got = cam["rgb"].get_data()
                if got is None:
                    continue
                img = np.asarray(got)[..., :3].astype(np.float32) / 255.0
                # (H,W,C) -> (1,C,H,W)
                t = torch.from_numpy(img).permute(2, 0, 1).unsqueeze(0).to(dev)
                batch[f"observation.images.{cam['name']}"] = t

            # 3) 归一化 → 前向 → 反归一化
            if _STATE["pre"] is not None:
                batch = _STATE["pre"](batch)
            with torch.no_grad():
                a = _STATE["policy"].select_action(batch)
            if _STATE["post"] is not None:
                out = _STATE["post"]({"action": a})
                a = out.get("action", a) if isinstance(out, dict) else out
            act = np.asarray(a.squeeze(0).detach().cpu(), dtype=np.float32).reshape(-1)

            # 4) 下发关节目标（策略输出 32 维，正好对应全部 DOF）
            if act.size != len(_STATE["dof_names"]):
                raise RuntimeError(
                    f"动作维度 {act.size} 与 DOF 数 {len(_STATE['dof_names'])} 不匹配"
                    "（策略和机器人不是同一套关节？）"
                )
            _STATE["art"].set_dof_position_targets(
                act.astype(np.float32).reshape(1, -1)
            )
            _STATE["frame"] += 1
            if _STATE["frame"] % 50 == 0:
                _log(f"frame {_STATE['frame']} 动作范围 "
                     f"[{act.min():.3f}, {act.max():.3f}]")
            if _STATE["frame"] >= _STATE["max_frames"]:
                _log("达到 max_frames，自动停止")
                stop()
        except Exception as exc:  # noqa: BLE001
            _STATE["errors"] += 1
            if _STATE["errors"] == 1:
                import traceback

                _log(f"推理失败（首次）: {exc!r}")
                traceback.print_exc()
            if _STATE["errors"] > 10:
                _log("错误过多，自动停止")
                stop()

    _STATE["sub"] = (
        omni.kit.app.get_app()
        .get_update_event_stream()
        .create_subscription_to_pop(_on_update, name="sdg_live_infer")
    )
    _log(f"已装载 —— 最多 {max_frames} 帧，开始 Play 后机器人会实时动起来")
    return {"ok": True, "cameras": [c["name"] for c in cams],
            "max_frames": int(max_frames), "device": str(dev)}
