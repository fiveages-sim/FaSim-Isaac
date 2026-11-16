"""实时体检：确认推理时的三路相机与训练数据完全对得上。

## 检查什么
| 项 | 判据 | 为什么重要 |
|---|---|---|
| 分辨率 | 每路 == 数据集 feature shape | 不一致 → FOV 不同 → 策略看不懂 |
| 通道顺序 | 与数据集图逐通道均值同序 | R/B 互换会让颜色全反 |
| 构图 | mean\\|diff\\| 足够小 | 反映 FOV/裁切/摆放是否一致 |
| 物体位置 | 试管处是否对齐 | 策略重点看试管（实测该路影响最大） |

**注意**：本脚本比较的是「当前场景」与「数据集第 N 帧」。
物体位置天然会有差异（采集时随机化过），所以 `mean|diff|` 不可能为 0。
判据看**分辨率/通道**这类硬指标，构图差异只作参考（会给出"物体位移"提示）。

## 用法（Isaac 在跑、Camera=enable）
    cd ~/lerobot_ros2 && source .venv-train/bin/activate
    python ~/FaSim-Isaac/exts/fasim.replicator.sdg/scripts/check_camera_live.py
"""

from __future__ import annotations

import argparse
import json
from pathlib import Path

import numpy as np

TOPICS = {"head": "/head_camera/rgb",
          "wrist_left": "/left_hand_camera/rgb",
          "wrist_right": "/right_hand_camera/rgb"}
OUT = Path("/tmp/sdg_cam_live_check")


def grab(timeout: float = 6.0):
    import time

    import rclpy
    from rclpy.node import Node
    from rosidl_runtime_py.utilities import get_message

    rclpy.init()
    node = Node("sdg_cam_check")
    got: dict[str, np.ndarray] = {}
    meta: dict[str, str] = {}
    types = dict(node.get_topic_names_and_types())

    def mk(name):
        def cb(msg):
            if name in got:
                return
            h, w = int(msg.height), int(msg.width)
            enc = (msg.encoding or "").lower()
            ch = {"rgb8": 3, "bgr8": 3, "rgba8": 4, "bgra8": 4}.get(enc, 3)
            buf = np.frombuffer(bytes(msg.data), dtype=np.uint8)
            if buf.size >= h * w * ch:
                got[name] = buf[: h * w * ch].reshape(h, w, ch)[..., :3].copy()
                meta[name] = enc
        return cb

    subs = [node.create_subscription(get_message(types[t][0]), t, mk(n), 10)
            for n, t in TOPICS.items() if t in types]
    missing_topics = [n for n, t in TOPICS.items() if t not in types]
    t0 = time.monotonic()
    while len(got) < len(TOPICS) - len(missing_topics) and time.monotonic() - t0 < timeout:
        rclpy.spin_once(node, timeout_sec=0.1)
    for s in subs:
        node.destroy_subscription(s)
    node.destroy_node()
    rclpy.shutdown()
    return got, meta, missing_topics


def ds_frames(root: Path, episode: int, frame: int) -> dict[str, np.ndarray]:
    from lerobot.datasets.lerobot_dataset import LeRobotDataset

    ds = LeRobotDataset(repo_id=root.name, root=str(root), video_backend="pyav")
    ep = np.asarray(ds.hf_dataset["episode_index"])
    idx = np.flatnonzero(ep == episode)
    it = ds[int(idx[min(frame, len(idx) - 1)])]
    out = {}
    for k in ds.meta.camera_keys:
        a = it[k]
        a = a.numpy() if hasattr(a, "numpy") else np.asarray(a)
        if a.ndim == 3 and a.shape[0] in (1, 3, 4):
            a = np.transpose(a, (1, 2, 0))
        if a.dtype != np.uint8:
            a = (np.clip(a, 0, 1) * 255).astype(np.uint8)
        out[k.replace("observation.images.", "")] = a[..., :3]
    return out


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--dataset", default=str(Path.home() / "lerobot_datasets/tube_ee"))
    ap.add_argument("--episode", type=int, default=0)
    ap.add_argument("--frame", type=int, default=0)
    args = ap.parse_args()
    OUT.mkdir(parents=True, exist_ok=True)

    print("=" * 70)
    print("实时相机体检")
    print("=" * 70)

    live, enc_meta, missing = grab()
    if missing:
        print(f"!! 缺话题: {missing}")

    ds_imgs = ds_frames(Path(args.dataset).expanduser(), args.episode, args.frame)

    rows = []
    fails: list[str] = []
    for name in ("head", "wrist_left", "wrist_right"):
        ref = ds_imgs.get(name)
        cur = live.get(name)
        if ref is None:
            fails.append(f"{name}: 数据集里没有这一路")
            continue
        if cur is None:
            fails.append(f"{name}: 话题没有图像")
            continue

        ds_hw = tuple(ref.shape[:2])
        lv_hw = tuple(cur.shape[:2])
        enc = enc_meta.get(name, "?")
        same_hw = ds_hw == lv_hw

        # 通道顺序：比较三通道均值的相对大小关系（R>G>B vs B>G>R）
        dm = [float(ref[..., i].mean()) for i in range(3)]
        lm = [float(cur[..., i].mean()) for i in range(3)]
        order_ds = tuple(np.argsort(dm))
        order_lv = tuple(np.argsort(lm))
        same_order = order_ds == order_lv

        # 构图差异（resize 到同尺寸后）
        import cv2
        a = cur if same_hw else cv2.resize(cur, (ds_hw[1], ds_hw[0]),
                                           interpolation=cv2.INTER_AREA)
        d = np.abs(a.astype(np.int16) - ref.astype(np.int16))
        mad = float(d.mean())
        # 差异热区重心 —— 判断差异集中在画面中心（物体）还是全画面（FOV）
        ys, xs = np.nonzero(d.mean(axis=2) > 20)
        if len(ys):
            cy, cx = float(ys.mean()) / ds_hw[0], float(xs.mean()) / ds_hw[1]
            spread = float(d.mean(axis=2).std())
        else:
            cy = cx = 0.5
            spread = 0.0

        ok = same_hw and same_order
        rows.append({
            "camera": name, "ds_hw": list(ds_hw), "live_hw": list(lv_hw),
            "encoding": enc, "same_resolution": same_hw,
            "ds_ch_mean": [round(v, 1) for v in dm],
            "live_ch_mean": [round(v, 1) for v in lm],
            "same_channel_order": same_order,
            "mean_abs_diff": round(mad, 2),
            "diff_centroid": [round(cy, 2), round(cx, 2)],
            "diff_spread": round(spread, 2),
            "ok": ok,
        })
        if not same_hw:
            fails.append(f"{name}: 分辨率 {lv_hw} != 数据集 {ds_hw}")
        if not same_order:
            fails.append(f"{name}: 通道顺序不一致（疑似 BGR/RGB 反了）")

        print()
        print(f"[{name}] {'OK' if ok else 'FAIL'}")
        print(f"  分辨率   数据集 {ds_hw}  实时 {lv_hw}  "
              f"{'✅ 一致' if same_hw else '❌ 不一致'}")
        print(f"  编码     实时 {enc}")
        print(f"  通道均值 数据集 {[round(v,1) for v in dm]}  实时 {[round(v,1) for v in lm]}  "
              f"{'✅ 同序' if same_order else '❌ 反序'}")
        print(f"  构图差异 mean|diff|={mad:.2f}  "
              f"差异重心=(y{cy:.2f}, x{cx:.2f}) 扩散={spread:.2f}")

        # 保存对比图（拼图：数据集 | 实时 | 差异）
        try:
            import cv2
            tiles = []
            for img, label in ((ref, "dataset"), (a, "live"),
                               (np.clip(d * 3, 0, 255).astype(np.uint8), "|diff|x3")):
                t = cv2.resize(img, (320, 240), interpolation=cv2.INTER_AREA)
                t = cv2.copyMakeBorder(t, 20, 2, 2, 2, cv2.BORDER_CONSTANT, value=(30, 30, 30))
                cv2.putText(t, label, (4, 14), cv2.FONT_HERSHEY_SIMPLEX, 0.42,
                            (255, 255, 255), 1, cv2.LINE_AA)
                tiles.append(t)
            cv2.imwrite(str(OUT / f"{name}.png"),
                        cv2.cvtColor(np.hstack(tiles), cv2.COLOR_RGB2BGR))
        except Exception:
            pass

    print()
    print("=" * 70)
    print("结论")
    print("=" * 70)
    if fails:
        print("  ❌ 有硬性问题（必须先修）：")
        for f in fails:
            print(f"     - {f}")
    else:
        print("  ✅ 三路相机的分辨率与通道顺序全部与训练数据一致")

    # 构图提示（不是硬失败 —— 物体位置天然会不同）
    warn = [r for r in rows if r["mean_abs_diff"] > 15]
    if warn:
        print()
        print("  构图差异偏大（可能是物体摆放位置与数据集该帧不同，属正常）：")
        for r in warn:
            print(f"     - {r['camera']}: mean|diff|={r['mean_abs_diff']}, "
                  f"差异重心=(y{r['diff_centroid'][0]}, x{r['diff_centroid'][1]})")
        print("     若重心在画面中部且扩散小 → 是物体位移；若铺满全画面 → 是视角/光照差异")

    (OUT / "report.json").write_text(
        json.dumps({"cameras": rows, "fails": fails,
                    "encoding": enc_meta}, indent=2, ensure_ascii=False),
        encoding="utf-8")
    print()
    print(f"报告: {OUT}/report.json")
    print(f"对比图: {OUT}/{{head,wrist_left,wrist_right}}.png")
    return 0 if not fails else 2


if __name__ == "__main__":
    raise SystemExit(main())
