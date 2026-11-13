"""LeRobot 导出的 worker —— 在**装了 lerobot 的 python** 里运行。

为什么拆成子进程
================
主进程（sdg.sh 用的系统 python）只有 numpy/PyYAML；而写 LeRobot 数据集需要
pyarrow/torch/av 等重依赖。把它拆成子进程，主进程只负责探测环境 + 传 JSON
请求，避免给采集侧引入重依赖。

输入：一个 JSON 文件路径（参数与帧文件列表）。
输出：在 ``out`` 目录生成 LeRobot 数据集；stdout 打进度。
"""

from __future__ import annotations

import json
import sys
from pathlib import Path

import numpy as np


def _log(msg: str) -> None:
    print(f"[export] {msg}", flush=True)


def _img(a) -> np.ndarray | None:
    arr = np.asarray(a)
    if arr.ndim == 3 and arr.shape[2] >= 3:
        arr = arr[..., :3]
    elif arr.ndim == 2:
        arr = np.stack([arr] * 3, axis=-1)
    else:
        return None
    if arr.dtype != np.uint8:
        arr = np.clip(arr, 0, 255).astype(np.uint8)
    return np.ascontiguousarray(arr)


def _quat_delta(q_from: np.ndarray, q_to: np.ndarray) -> np.ndarray:
    """两四元数之间的**增量**（xyzw），满足 q_to ≈ q_from ⊗ delta。

    同事数据集用 ``*_dquat_*`` 表示这个量（unit quaternion，实测模长 1.0000）。
    约定与他们的实现一致：delta = conj(q_from) ⊗ q_to，符号取与 q_from
    同半球（w>=0 的等价旋转），避免 q/-q 跳变。
    """
    a = np.asarray(q_from, dtype=np.float64).reshape(4)
    b = np.asarray(q_to, dtype=np.float64).reshape(4)
    # 归一化，避免累积误差（采集值应已是 unit）
    na, nb = np.linalg.norm(a), np.linalg.norm(b)
    if na > 0:
        a = a / na
    if nb > 0:
        b = b / nb
    # 让 b 与 a 同半球，避免走远路
    if float(np.dot(a, b)) < 0:
        b = -b
    ax, ay, az, aw = a
    bx, by, bz, bw = b
    # conj(a) ⊗ b
    dx = aw * bx - ax * bw - ay * bz + az * by
    dy = aw * by + ax * bz - ay * bw - az * bx
    dz = aw * bz - ax * by + ay * bx - az * bw
    dw = aw * bw + ax * bx + ay * by + az * bz
    d = np.array([dx, dy, dz, dw], dtype=np.float64)
    n = np.linalg.norm(d)
    return (d / n) if n > 0 else np.array([0.0, 0.0, 0.0, 1.0])


def main(req_path: str) -> int:
    req = json.loads(Path(req_path).read_text(encoding="utf-8"))

    try:
        from lerobot.datasets.lerobot_dataset import LeRobotDataset
    except ImportError as exc:
        _log(f"ERROR 这个 python 里没有 lerobot: {exc}")
        return 2

    frames = [Path(p) for p in req["frames"]]
    cams: list[str] = req["cams"]
    out = Path(req["out"])
    repo = req["repo_id"]
    fps = int(req["fps"])
    task = req["task"]
    robot_type = req["robot_type"]
    features = req["features"]
    use_videos = bool(req.get("use_videos", True))

    # LeRobot 的 features 里 names 不能是 None（某些版本会崩），统一去掉空值
    clean: dict = {}
    for k, v in features.items():
        vv = dict(v)
        if vv.get("names") is None:
            vv.pop("names", None)
        clean[k] = vv

    _log(f"创建数据集: {repo} @ {out}  fps={fps}  features={sorted(clean)}")
    ds = LeRobotDataset.create(
        repo_id=repo,
        fps=fps,
        features=clean,
        robot_type=robot_type,
        root=out,
        use_videos=use_videos,
    )

    # **绕过 LeRobot v3.0 的一个校验 bug**：`create()` 把 features 写进
    # info.json 时 shape 变成 JSON 数组，内存里的 `ds.features[k]["shape"]`
    # 因此是 **list**；而 `validate_feature_numpy_array` 拿它跟
    # `np.ndarray.shape`（**tuple**）比较 —— `[32] != (32,)` 恒为 True，
    # 导致任何帧都过不了校验。
    #
    # 官方示例走 `hw_to_dataset_features()` 直接构造、不经 JSON 往返，
    # 所以没暴露这个问题。这里把内存里的 shape 转回 tuple 即可。
    for _fk, _fv in (ds.features or {}).items():
        if isinstance(_fv.get("shape"), list):
            _fv["shape"] = tuple(_fv["shape"])
        # names 也统一成 list（某些校验按 list 比较）
        if isinstance(_fv.get("names"), tuple):
            _fv["names"] = list(_fv["names"])

    n_ok = 0
    n_missing_cmd = 0
    # ``episodes``: [[frame_path, ...], ...] —— 每个子列表 = 一个 episode。
    # 兼容旧的单 episode 请求（只有 ``frames`` 扁平列表 → 包成一层）。
    episodes = req.get("episodes")
    if episodes is None:
        episodes = [frames]
    total_frames = sum(len(e) for e in episodes)
    if len(episodes) > 1:
        _log(f"多 episode 导出：{len(episodes)} 组 / 共 {total_frames} 帧")

    asp = str(req.get("action_space") or "absolute").lower()
    done_frames = 0

    for ep_i, ep_frames in enumerate(episodes):
        if len(episodes) > 1:
            _log(f"--- episode {ep_i}（{len(ep_frames)} 帧）")
        ep_ok = 0

        for i, fp in enumerate(ep_frames):
            try:
                with np.load(fp, allow_pickle=False) as z:
                    keys = set(z.files)
                    frame: dict = {}

                    # ---- observation.state ← joint_pos（实测）----
                    if "joint_pos" in keys:
                        frame["observation.state"] = np.asarray(
                            z["joint_pos"], dtype=np.float32
                        ).reshape(-1)

                    # ---- action：三种语义 ----
                    #   absolute（默认）: joint_pos_cmd —— 绝对关节指令。
                    #       部署时策略直接输出目标角，无累积误差。
                    #   delta           : joint_pos[t+1] - joint_pos[t]（相邻实测差）。
                    #       满足 state[t]+action[t]==state[t+1]，与同事数据集同语义。
                    #       注意：不是 cmd−实测（那是控制器修正量，实测大 7.4 倍）。
                    #   ee_delta        : 末端相邻增量 14 维（对齐同事 pose 模式）。
                    if asp == "delta":
                        if "joint_pos" in keys:
                            q_now = np.asarray(z["joint_pos"], dtype=np.float32).reshape(-1)
                            nxt = None
                            # 注意：用 **ep_frames**（本 episode 内），
                            # 不能跨 episode 取下一帧，否则边界会串到别的轨迹。
                            if i + 1 < len(ep_frames):
                                with np.load(ep_frames[i + 1], allow_pickle=False) as z2:
                                    if "joint_pos" in z2.files:
                                        nxt = np.asarray(
                                            z2["joint_pos"], dtype=np.float32
                                        ).reshape(-1)
                            if nxt is not None and nxt.shape == q_now.shape:
                                frame["action"] = (nxt - q_now).astype(np.float32)
                            else:
                                # episode 最后一帧没有 t+1，填零（下面统计告警）
                                frame["action"] = np.zeros_like(q_now)
                                n_missing_cmd += 1
                    elif asp == "ee_delta":
                        ad: list = []
                        ok = i + 1 < len(ep_frames)
                        if ok:
                            with np.load(ep_frames[i + 1], allow_pickle=False) as z2:
                                nkeys = set(z2.files)
                                for side in ("left", "right"):
                                    pk, qk = f"{side}_pos", f"{side}_quat"
                                    if not (pk in keys and qk in keys
                                            and pk in nkeys and qk in nkeys):
                                        ok = False
                                        break
                                    p_now = np.asarray(z[pk], dtype=np.float32).reshape(-1)[:3]
                                    q_now = np.asarray(z[qk], dtype=np.float32).reshape(-1)[:4]
                                    p_nxt = np.asarray(z2[pk], dtype=np.float32).reshape(-1)[:3]
                                    q_nxt = np.asarray(z2[qk], dtype=np.float32).reshape(-1)[:4]
                                    ad.extend((p_nxt - p_now).astype(np.float32))
                                    ad.extend(_quat_delta(q_now, q_nxt).astype(np.float32))
                        if ok and len(ad) == 14:
                            frame["action"] = np.asarray(ad, dtype=np.float32)
                        else:
                            frame["action"] = np.zeros(14, dtype=np.float32)
                            n_missing_cmd += 1
                    else:  # absolute
                        akey = "joint_pos_cmd" if "joint_pos_cmd" in keys else "joint_pos"
                        if akey in keys:
                            frame["action"] = np.asarray(
                                z[akey], dtype=np.float32
                            ).reshape(-1)

                    # ---- 末端实测 / 指令 ----
                    if "observation.ee" in clean:
                        ee = []
                        for side in ("left", "right"):
                            pk, qk = f"{side}_pos", f"{side}_quat"
                            if pk in keys and qk in keys:
                                ee.extend(np.asarray(z[pk], dtype=np.float32).reshape(-1)[:3])
                                ee.extend(np.asarray(z[qk], dtype=np.float32).reshape(-1)[:4])
                        if ee:
                            frame["observation.ee"] = np.asarray(ee, dtype=np.float32)
                    if "observation.ee_cmd" in clean:
                        eec = []
                        for side in ("left", "right"):
                            pk, qk = f"{side}_cmd_pos", f"{side}_cmd_quat"
                            if pk in keys and qk in keys:
                                eec.extend(np.asarray(z[pk], dtype=np.float32).reshape(-1)[:3])
                                eec.extend(np.asarray(z[qk], dtype=np.float32).reshape(-1)[:4])
                        if eec:
                            frame["observation.ee_cmd"] = np.asarray(eec, dtype=np.float32)

                    # ---- 图像 ----
                    for c in cams:
                        rk = f"{c}_rgb"
                        if rk in keys:
                            im = _img(z[rk])
                            if im is not None:
                                frame[f"observation.images.{c}"] = im

                    # ---- 交互物体位姿（pos3 + quat4）----
                    for lab in (req.get("obj_labels") or []):
                        pk, qk = f"obj/{lab}_pos", f"obj/{lab}_quat"
                        if pk in keys and qk in keys:
                            frame[f"observation.objects.{lab}"] = np.concatenate([
                                np.asarray(z[pk], dtype=np.float32).reshape(-1)[:3],
                                np.asarray(z[qk], dtype=np.float32).reshape(-1)[:4],
                            ])

                frame["task"] = task
                ds.add_frame(frame)
                n_ok += 1
                ep_ok += 1
                done_frames += 1
            except Exception as exc:  # noqa: BLE001
                _log(f"WARN ep{ep_i} 第 {i} 帧失败: {exc}")
            if (i + 1) % 100 == 0:
                _log(f"  已写入 {ep_ok}/{len(ep_frames)} 帧（ep{ep_i}）")

        _log(f"保存 episode {ep_i}（{ep_ok} 帧）…")
        ds.save_episode()

    try:
        ds.finalize()  # v3 有 finalize（聚合 stats/视频元信息）；旧版没有
    except Exception:  # noqa: BLE001
        pass
    _log(f"完成: {out}  ({n_ok} 帧 / {len(episodes)} episode)")
    if asp != "absolute":
        _log(f"  action 语义: {asp}")
        if n_missing_cmd:
            _log(f"  WARN {n_missing_cmd} 帧无法构造 action（episode 末帧或数据缺失），"
                 f"已填零 —— 这些帧训练时应忽略")
    return 0


if __name__ == "__main__":
    if len(sys.argv) < 2:
        print("用法: _lerobot_export_worker.py <request.json>")
        sys.exit(2)
    sys.exit(main(sys.argv[1]))
