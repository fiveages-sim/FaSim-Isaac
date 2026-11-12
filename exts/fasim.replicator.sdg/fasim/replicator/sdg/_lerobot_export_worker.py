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
    for i, fp in enumerate(frames):
        try:
            with np.load(fp, allow_pickle=False) as z:
                keys = set(z.files)
                frame: dict = {}
                # observation.state ← joint_pos（实测）
                if "joint_pos" in keys:
                    frame["observation.state"] = np.asarray(
                        z["joint_pos"], dtype=np.float32
                    ).reshape(-1)
                # action ← joint_pos_cmd（**指令**）
                akey = "joint_pos_cmd" if "joint_pos_cmd" in keys else "joint_pos"
                if akey in keys:
                    frame["action"] = np.asarray(
                        z[akey], dtype=np.float32
                    ).reshape(-1)
                # 末端实测 / 指令
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
                # 图像
                for c in cams:
                    rk = f"{c}_rgb"
                    if rk in keys:
                        im = _img(z[rk])
                        if im is not None:
                            frame[f"observation.images.{c}"] = im
                frame["task"] = task
                ds.add_frame(frame)
                n_ok += 1
        except Exception as exc:  # noqa: BLE001
            _log(f"WARN 第 {i} 帧失败: {exc}")
        if (i + 1) % 100 == 0:
            _log(f"  已写入 {n_ok}/{len(frames)} 帧")

    _log(f"保存 episode（{n_ok} 帧）…")
    ds.save_episode()
    try:
        ds.finalize()  # v3 有 finalize（聚合 stats/视频元信息）；旧版没有
    except Exception:  # noqa: BLE001
        pass
    _log(f"完成: {out}  ({n_ok} 帧)")
    return 0


if __name__ == "__main__":
    if len(sys.argv) < 2:
        print("用法: _lerobot_export_worker.py <request.json>")
        sys.exit(2)
    sys.exit(main(sys.argv[1]))
