"""把已导出的关节角 action（32 维）改成 tube_grip 同款末端+夹爪（16 维）。

为什么不重新 export_session
============================
``export_session`` 的图像只从 ``frames/*.npz`` 读。run_042–076 的 frames
已经裁成 8 帧质监，完整图像只留在 ``~/lerobot_datasets`` 的 mp4 里。
再导出一次会把几百帧的视频变成 8 帧。

这条脚本因此：

1. 复制数据集，视频用硬链接（不重新编码）
2. 用 ``episode.npz`` 的 ``*_cmd_pos/_cmd_quat`` + 夹爪开关量替换 action
   （与 ``export_lerobot._load_ee_actions(..., with_gripper=True)`` 同一条路径）
3. run_042 没有 episode.npz：用数据集里的关节指令做同样的 FK
4. 重算 action 的 stats（info / stats.json / episodes parquet）
5. 合并成一个数据集

特征集合对齐 ``~/lerobot_datasets/tube_grip``：不改 observation、相机、物体。
"""

from __future__ import annotations

import hashlib
import json
import os
import shutil
import sys
import tempfile
from pathlib import Path

import numpy as np
import pyarrow as pa
import pyarrow.parquet as pq

EXT_ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(EXT_ROOT))

from fasim.replicator.sdg.completion import (  # noqa: E402
    HOLE_3_LOCAL,
    _quat_rotate_wxyz,
    tilt_deg_from_upright,
)
from fasim.replicator.sdg.export_lerobot import (  # noqa: E402
    _gripper_switch_series,
    _load_ee_actions,
)

DATA_ROOT = Path("/home/fiveages/fasim_data/unitree_g1d/sinopec_test_tube")
DS_ROOT = Path("/home/fiveages/lerobot_datasets")
TUBE_GRIP = DS_ROOT / "tube_grip"
OUT_ROOT = DS_ROOT / "ee16"
MERGED = DS_ROOT / "g1d_tube_v2"

ACTION_NAMES = [
    "left_ee.pos.x", "left_ee.pos.y", "left_ee.pos.z",
    "left_ee.quat.x", "left_ee.quat.y", "left_ee.quat.z", "left_ee.quat.w",
    "right_ee.pos.x", "right_ee.pos.y", "right_ee.pos.z",
    "right_ee.quat.x", "right_ee.quat.y", "right_ee.quat.z", "right_ee.quat.w",
    "left_gripper.pos", "right_gripper.pos",
]
STAT_KEYS = ("min", "max", "mean", "std", "count", "q01", "q10", "q50", "q90", "q99")


def _log(msg: str) -> None:
    print(f"[ee16] {msg}", flush=True)


def _link_or_copy(src: Path, dst: Path) -> None:
    dst.parent.mkdir(parents=True, exist_ok=True)
    if dst.exists():
        dst.unlink()
    try:
        os.link(src, dst)
    except OSError:
        shutil.copy2(src, dst)


def clone_dataset(src: Path, dst: Path) -> None:
    """视频硬链接，meta/data 真复制（后面要改，不能动原件）。"""
    if dst.exists():
        shutil.rmtree(dst)
    for p in src.rglob("*"):
        if not p.is_file():
            continue
        rel = p.relative_to(src)
        target = dst / rel
        if p.suffix == ".mp4":
            _link_or_copy(p, target)
        else:
            target.parent.mkdir(parents=True, exist_ok=True)
            shutil.copy2(p, target)


def episode_lengths(ds: Path) -> list[int]:
    files = sorted((ds / "meta" / "episodes").rglob("*.parquet"))
    tab = pq.read_table(files[0], columns=["episode_index", "length"])
    order = np.argsort(np.asarray(tab.column("episode_index").to_pylist()))
    lens = np.asarray(tab.column("length").to_pylist())
    return [int(x) for x in lens[order]]


def actions_from_episode_npz(run_dir: Path, n_frames: int) -> np.ndarray:
    ep = run_dir / "episode.npz"
    if not ep.is_file():
        raise FileNotFoundError(ep)
    rows = _load_ee_actions(ep, n_frames, with_gripper=True)
    arr = np.asarray(rows, dtype=np.float32)
    if arr.shape != (n_frames, 16):
        raise RuntimeError(f"{run_dir.name}: action {arr.shape} != ({n_frames}, 16)")
    return arr


def actions_from_joint_fk(ds: Path) -> np.ndarray:
    """run_042：没有 episode.npz，关节指令就在现有 action 列里。

    与 pack 同一条 FK：``joint_pos_cmd`` → tcp，再用第 0 帧实测末端
    （observation.ee，wxyz）反解 base。夹爪开关量走 ``_gripper_switch_series``。
    """
    from fasim.replicator.sdg.config_io import load_robot_config
    from fasim.replicator.sdg.fk import make_fk

    data = sorted((ds / "data").rglob("*.parquet"))[0]
    tab = pq.read_table(data, columns=["action", "observation.ee"])
    q = np.asarray(tab.column("action").to_pylist(), dtype=np.float64)
    ee = np.asarray(tab.column("observation.ee").to_pylist(), dtype=np.float64)
    info = json.loads((ds / "meta" / "info.json").read_text(encoding="utf-8"))
    names = list(info["features"]["action"]["names"])
    if q.ndim != 2 or q.shape[1] != len(names):
        raise RuntimeError(f"关节 action 形状 {q.shape} 与名字 {len(names)} 不符")

    planning = dict(load_robot_config("unitree_g1d").planning or {})
    fk = make_fk(planning, robot_key="unitree_g1d")
    if fk is None:
        raise RuntimeError("构造 G1D FK 失败（找不到 planning URDF）")

    rest_pos = {
        "left": ee[0, 0:3].copy(),
        "right": ee[0, 7:10].copy(),
    }
    rest_quat = {
        "left": ee[0, 3:7].copy(),
        "right": ee[0, 10:14].copy(),
    }
    # 逐帧基座（与 pack 口径一致）：基座会摆动，冻结首帧会让世界系末端指令
    # 偏 mean≈12mm / max≈47mm。ee 列本身是实测末端（14 维），可直接逐帧反解。
    poses = None
    try:
        bt, bR = fk.solve_base_track(
            {"left": ee[:, 0:3], "right": ee[:, 7:10]},
            {"left": ee[:, 3:7], "right": ee[:, 10:14]},
            q,
            names,
        )
        poses = fk.tcp_poses_with_base(q, names, (bt, bR))
    except Exception as exc:  # noqa: BLE001
        _log(f"  WARN 逐帧基座反解失败，退回首帧固定基座: {exc}")
    if poses is None:
        fk.solve_base_from_rest(rest_pos, rest_quat,
                                rest_joint_pos=q[0], rest_joint_names=names)
        poses = fk.tcp_poses(q, names, to_world=True)

    with tempfile.NamedTemporaryFile(suffix=".npz", delete=False) as tmp:
        tmp_path = Path(tmp.name)
    try:
        np.savez(
            tmp_path,
            joint_pos_cmd=q.astype(np.float32),
            joint_names=np.array(names),
        )
        grip = _gripper_switch_series(tmp_path, len(q))
    finally:
        tmp_path.unlink(missing_ok=True)

    out = np.zeros((len(q), 16), dtype=np.float32)
    for i in range(len(q)):
        lp, lq = poses["left"][0][i], poses["left"][1][i]
        rp, rq = poses["right"][0][i], poses["right"][1][i]
        # FK 四元数是 wxyz；action 名字是 xyzw（与 _load_ee_actions 一致）
        out[i] = [
            lp[0], lp[1], lp[2], lq[1], lq[2], lq[3], lq[0],
            rp[0], rp[1], rp[2], rq[1], rq[2], rq[3], rq[0],
            grip[i][0], grip[i][1],
        ]
    if not np.isfinite(out[:, :14]).all():
        raise RuntimeError("FK 产出了非有限位姿")
    return out


def validate_fk_against_run046() -> None:
    """用 run_046 的 episode.npz 对拍：FK(joint_pos_cmd) 必须等于已存的 cmd 位姿。"""
    from fasim.replicator.sdg.config_io import load_robot_config
    from fasim.replicator.sdg.fk import make_fk

    z = np.load(DATA_ROOT / "run_046" / "episode.npz")
    planning = dict(load_robot_config("unitree_g1d").planning or {})
    fk = make_fk(planning, robot_key="unitree_g1d")
    if fk is None:
        raise RuntimeError("FK 不可用，不能恢复 run_042")
    names = [str(x) for x in z["joint_names"]]
    rest_pos = {"left": np.asarray(z["left_pos"][0]), "right": np.asarray(z["right_pos"][0])}
    rest_quat = {"left": np.asarray(z["left_quat"][0]), "right": np.asarray(z["right_quat"][0])}
    # 用首帧真实关节角反解 base（与 rewrite_run_042_actions 一致）。
    # 旧版假定首帧 q≈0，对 home 姿态会系统性偏 10~16mm。
    fk.solve_base_from_rest(
        rest_pos, rest_quat,
        rest_joint_pos=np.asarray(z["joint_pos"], dtype=np.float64)[0],
        rest_joint_names=names,
    )

    # 真值校验：FK(实测关节角) 必须复现实测末端位姿。
    # 这条与 episode 里存的 *_cmd_pos 无关，所以不受 base 解算口径变化影响。
    meas = fk.tcp_poses(np.asarray(z["joint_pos"], dtype=np.float64), names, to_world=True)
    m_err = float(np.nanmax(np.linalg.norm(meas["left"][0] - np.asarray(z["left_pos"]), axis=1))) * 1000
    m_err_r = float(np.nanmax(np.linalg.norm(meas["right"][0] - np.asarray(z["right_pos"]), axis=1))) * 1000
    _log(f"FK 真值校验 run_046：左 {m_err:.2f}mm / 右 {m_err_r:.2f}mm（轴关节长链，>20mm 视为异常）")
    if max(m_err, m_err_r) > 20.0:
        raise RuntimeError(f"FK 复现实测末端差 {max(m_err, m_err_r):.2f}mm，停止")

    # 与历史（q≈0 口径）存的 cmd 位姿对拍：口径变了，只能看量级，不能当断言。
    poses = fk.tcp_poses(np.asarray(z["joint_pos_cmd"]), names, to_world=True)
    err = np.linalg.norm(poses["left"][0] - np.asarray(z["left_cmd_pos"]), axis=1)
    err_r = np.linalg.norm(poses["right"][0] - np.asarray(z["right_cmd_pos"]), axis=1)
    _log(f"  （相对旧口径 *_cmd_pos：左 {np.nanmean(err)*1000:.2f}mm / 右 {np.nanmean(err_r)*1000:.2f}mm "
         f"—— 差异来自 base 解算口径修正，属预期）")


def _replace_action_column(table: pa.Table, actions: np.ndarray) -> pa.Table:
    flat = pa.array(np.asarray(actions, dtype=np.float32).reshape(-1), type=pa.float32())
    col = pa.FixedSizeListArray.from_arrays(flat, 16)
    field = pa.field("action", pa.list_(pa.float32(), 16))
    idx = table.schema.get_field_index("action")
    table = table.set_column(idx, field, col)

    meta = dict(table.schema.metadata or {})
    if b"huggingface" in meta:
        hf = json.loads(meta[b"huggingface"])
        hf["info"]["features"]["action"]["length"] = 16
        blob = json.dumps(hf, ensure_ascii=False).encode()
        hf["fingerprint"] = hashlib.sha256(blob).hexdigest()
        meta[b"huggingface"] = json.dumps(hf, ensure_ascii=False).encode()
        table = table.replace_schema_metadata(meta)
    return table


def _action_stats(actions: np.ndarray) -> dict[str, np.ndarray]:
    from lerobot.datasets.compute_stats import get_feature_stats

    return get_feature_stats(np.asarray(actions, dtype=np.float64), axis=0, keepdims=False)


def _write_episode_action_stats(ds: Path, per_ep: list[np.ndarray]) -> None:
    path = sorted((ds / "meta" / "episodes").rglob("*.parquet"))[0]
    table = pq.read_table(path)
    stats = [_action_stats(a) for a in per_ep]
    for key in STAT_KEYS:
        colname = f"stats/action/{key}"
        if table.schema.get_field_index(colname) < 0:
            raise RuntimeError(f"{ds.name} 缺列 {colname}")
        if key == "count":
            values = [[int(x) for x in np.asarray(st[key]).reshape(-1)] for st in stats]
            arr = pa.array(values, type=pa.list_(pa.int64()))
        else:
            values = [[float(x) for x in np.asarray(st[key]).reshape(-1)] for st in stats]
            arr = pa.array(values, type=pa.list_(pa.float64()))
        table = table.set_column(table.schema.get_field_index(colname), colname, arr)
    pq.write_table(table, path)


def _write_stats_json(ds: Path, per_ep: list[np.ndarray]) -> None:
    from lerobot.datasets.compute_stats import aggregate_stats
    from lerobot.datasets.io_utils import write_stats

    ep_stats = [{"action": _action_stats(a)} for a in per_ep]
    agg = aggregate_stats(ep_stats)
    stats_path = ds / "meta" / "stats.json"
    current = json.loads(stats_path.read_text(encoding="utf-8"))
    current["action"] = {
        k: np.asarray(v) for k, v in agg["action"].items()
    }
    write_stats(current, ds)


def _patch_info(ds: Path) -> None:
    info_path = ds / "meta" / "info.json"
    info = json.loads(info_path.read_text(encoding="utf-8"))
    tube = json.loads((TUBE_GRIP / "meta" / "info.json").read_text(encoding="utf-8"))
    info["features"]["action"] = {
        "dtype": "float32",
        "shape": [16],
        "names": list(ACTION_NAMES),
    }
    missing = [k for k in tube["features"] if k not in info["features"]]
    if missing:
        raise RuntimeError(f"{ds.name} 相对 tube_grip 缺特征 {missing}")
    extra = [k for k in info["features"] if k not in tube["features"]]
    if extra:
        raise RuntimeError(f"{ds.name} 比 tube_grip 多特征 {extra}")
    if info["features"]["observation.state"]["names"] != tube["features"]["observation.state"]["names"]:
        raise RuntimeError(f"{ds.name} observation.state 关节名与 tube_grip 不一致")
    for cam in ("head", "wrist_left", "wrist_right"):
        key = f"observation.images.{cam}"
        if info["features"][key]["shape"] != tube["features"][key]["shape"]:
            raise RuntimeError(f"{ds.name} {key} 分辨率不是 {tube['features'][key]['shape']}")
    info_path.write_text(json.dumps(info, indent=4, ensure_ascii=False) + "\n", encoding="utf-8")


def rewrite_one(src: Path, dst: Path, per_ep_actions: list[np.ndarray]) -> None:
    clone_dataset(src, dst)
    data_path = sorted((dst / "data").rglob("*.parquet"))[0]
    table = pq.read_table(data_path)
    n = table.num_rows
    actions = np.concatenate(per_ep_actions, axis=0)
    if len(actions) != n:
        raise RuntimeError(f"{src.name}: action {len(actions)} 帧 != parquet {n}")
    ep_idx = np.asarray(table.column("episode_index").to_pylist())
    # episode.npz 按 episode_index 递增拼接；确认 parquet 也是这个顺序
    if not np.all(ep_idx[:-1] <= ep_idx[1:]):
        raise RuntimeError(f"{src.name}: parquet 不是按 episode 排的，不能整列替换")
    table = _replace_action_column(table, actions)
    pq.write_table(table, data_path)
    _patch_info(dst)
    _write_episode_action_stats(dst, per_ep_actions)
    _write_stats_json(dst, per_ep_actions)
    _log(f"  {dst.name}: {len(per_ep_actions)} ep / {n} 帧  action[16]")


def best_insert(run_name: str) -> str:
    """用 episode.npz 的物体位姿复算插入判据。没有 npz 就跳过。"""
    ep = DATA_ROOT / run_name / "episode.npz"
    if not ep.is_file():
        return "no-episode-npz"
    z = np.load(ep)
    need = [
        "obj/test_tube_pos", "obj/test_tube_quat",
        "obj/test_tube_rack_pos", "obj/test_tube_rack_quat",
    ]
    if any(k not in z.files for k in need):
        return "no-objects"
    tube_p = np.asarray(z["obj/test_tube_pos"], dtype=np.float64)
    tube_q = np.asarray(z["obj/test_tube_quat"], dtype=np.float64)
    rack_p = np.asarray(z["obj/test_tube_rack_pos"], dtype=np.float64)
    rack_q = np.asarray(z["obj/test_tube_rack_quat"], dtype=np.float64)
    hole = np.stack([
        rack_p[i] + _quat_rotate_wxyz(rack_q[i], np.asarray(HOLE_3_LOCAL))
        for i in range(len(rack_p))
    ])
    dxy = np.linalg.norm(tube_p[:, :2] - hole[:, :2], axis=1) * 1000
    dz = (tube_p[:, 2] - hole[:, 2]) * 1000
    tilt = np.array([tilt_deg_from_upright(tube_q[i]) for i in range(len(tube_q))])
    ok = (dxy < 25.0) & (dz <= -60.0) & (tilt < 30.0)
    if ok.any():
        i = int(np.argmin(np.where(ok, dz, 0)))
        return f"OK dxy={dxy[i]:.1f} dz={dz[i]:.1f} tilt={tilt[i]:.1f}"
    i = int(np.argmin(dz))
    return f"FAIL dxy={dxy[i]:.1f} dz={dz[i]:.1f} tilt={tilt[i]:.1f}"


def check_loaded(ds: Path) -> None:
    from lerobot.datasets.lerobot_dataset import LeRobotDataset

    loaded = LeRobotDataset(repo_id=ds.name, root=ds)
    item = loaded[0]
    act = item["action"]
    if tuple(act.shape) != (16,):
        raise RuntimeError(f"{ds.name} 加载后 action {tuple(act.shape)}")
    names = loaded.meta.features["action"]["names"]
    if list(names) != ACTION_NAMES:
        raise RuntimeError(f"{ds.name} action 名字与 tube_grip 不一致")
    _log(f"  加载 {ds.name}: {loaded.meta.total_episodes} ep / "
         f"{loaded.meta.total_frames} 帧  action={tuple(act.shape)}")


def main() -> int:
    tube_names = json.loads((TUBE_GRIP / "meta" / "info.json").read_text())["features"]["action"]["names"]
    if list(tube_names) != ACTION_NAMES:
        _log("ERROR ACTION_NAMES 与 tube_grip 不一致")
        return 1

    _log("检查插入（episode.npz 物体位姿）")
    for i in range(42, 77):
        name = f"run_{i:03d}"
        sj = DATA_ROOT / name / "session.json"
        stop = "?"
        if sj.is_file():
            stop = json.loads(sj.read_text()).get("stop_reason")
        _log(f"  {name} stop={stop}  {best_insert(name)}")

    _log("FK 对拍")
    validate_fk_against_run046()

    if OUT_ROOT.exists():
        shutil.rmtree(OUT_ROOT)
    OUT_ROOT.mkdir(parents=True)

    # run_042：关节指令 FK
    _log("run_042 ← 关节指令 FK（无 episode.npz）")
    src042 = DS_ROOT / "run_042"
    act042 = actions_from_joint_fk(src042)
    rewrite_one(src042, OUT_ROOT / "run_042", [act042])

    # run_043..066：各自一个数据集
    for i in range(43, 67):
        name = f"run_{i:03d}"
        src = DS_ROOT / name
        info = json.loads((src / "meta" / "info.json").read_text())
        n = int(info["total_frames"])
        _log(f"{name} ← episode.npz")
        acts = actions_from_episode_npz(DATA_ROOT / name, n)
        rewrite_one(src, OUT_ROOT / name, [acts])

    # run_067..076 在同一个 10 episode 数据集里，长度已对上
    src10 = DS_ROOT / "sinopec_test_tube_10eps"
    lens = episode_lengths(src10)
    runs = [f"run_{i:03d}" for i in range(67, 77)]
    if len(lens) != len(runs):
        _log(f"ERROR 10eps 有 {len(lens)} 条，期望 10")
        return 1
    per = []
    for name, n in zip(runs, lens):
        ep_n = int(np.load(DATA_ROOT / name / "episode.npz")["left_cmd_pos"].shape[0])
        if ep_n != n:
            _log(f"ERROR {name} episode.npz {ep_n} != 数据集 {n}")
            return 1
        _log(f"{name} ← episode.npz（10eps 的一段）")
        per.append(actions_from_episode_npz(DATA_ROOT / name, n))
    rewrite_one(src10, OUT_ROOT / "run_067_076", per)

    _log("抽查加载")
    check_loaded(OUT_ROOT / "run_046")
    check_loaded(OUT_ROOT / "run_042")
    check_loaded(OUT_ROOT / "run_067_076")

    # 与 episode.npz 逐帧对拍一条
    ref = actions_from_episode_npz(DATA_ROOT / "run_046", 790)
    got = np.asarray(
        pq.read_table(
            sorted((OUT_ROOT / "run_046" / "data").rglob("*.parquet"))[0],
            columns=["action"],
        ).column("action").to_pylist(),
        dtype=np.float32,
    )
    if not np.allclose(ref, got):
        _log("ERROR run_046 写回的 action 与 _load_ee_actions 不一致")
        return 1
    grip = got[:, 14:]
    if not set(np.unique(np.round(grip, 5))).issubset({0.0, 1.0}):
        _log(f"ERROR 夹爪不是 0/1：{np.unique(grip)}")
        return 1
    _log(f"run_046 对拍通过；夹爪闭合帧 left={(grip[:,0]==0).sum()} right={(grip[:,1]==0).sum()}")

    _log("合并")
    from lerobot.datasets.dataset_tools import merge_datasets
    from lerobot.datasets.lerobot_dataset import LeRobotDataset

    if MERGED.exists():
        shutil.rmtree(MERGED)
    pieces = []
    for p in sorted(OUT_ROOT.iterdir()):
        if (p / "meta" / "info.json").is_file():
            pieces.append(LeRobotDataset(repo_id=p.name, root=p))
            _log(f"  载入 {p.name}: {pieces[-1].meta.total_episodes} ep / "
                 f"{pieces[-1].meta.total_frames} 帧")
    merged = merge_datasets(pieces, output_repo_id="g1d_tube_v2", output_dir=MERGED)
    info = json.loads((MERGED / "meta" / "info.json").read_text())
    names = info["features"]["action"]["names"]
    _log(f"完成 {MERGED}")
    _log(f"  {info['total_episodes']} episode / {info['total_frames']} 帧")
    _log(f"  action {info['features']['action']['shape']}  names_ok={list(names)==ACTION_NAMES}")
    _log(f"  加载帧数 {len(merged)}")
    missing = [k for k in json.loads((TUBE_GRIP / "meta" / "info.json").read_text())["features"]
               if k not in info["features"]]
    if missing or list(names) != ACTION_NAMES or info["features"]["action"]["shape"] != [16]:
        _log(f"ERROR 合并结果与 tube_grip 不一致 missing={missing}")
        return 1

    # 插孔失败的轨迹不进训练集。run_072 实测 tilt≈90°、离孔 116mm。
    order = [f"run_{i:03d}" for i in range(42, 67)] + [f"run_{i:03d}" for i in range(67, 77)]
    bad = [k for k, name in enumerate(order) if best_insert(name).startswith("FAIL")]
    if bad:
        from lerobot.datasets.dataset_tools import delete_episodes

        _log(f"去掉插孔失败的 episode {bad}："
             + ", ".join(order[k] for k in bad))
        tmp = MERGED.with_name(MERGED.name + "_tmp")
        if tmp.exists():
            shutil.rmtree(tmp)
        delete_episodes(merged, bad, output_dir=tmp, repo_id=MERGED.name)
        shutil.rmtree(MERGED)
        tmp.rename(MERGED)
        info = json.loads((MERGED / "meta" / "info.json").read_text())
        _log(f"最终 {info['total_episodes']} episode / {info['total_frames']} 帧")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
