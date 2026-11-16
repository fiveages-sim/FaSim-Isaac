"""运行时物体位姿随机化（配置驱动，不与具体机器人/任务强绑定）。

## 要解决什么
自动化采数据时，每轮之间要换一个场景布局，否则 100 条数据都是同一个配置，
策略学不到「泛化到不同物体位置」。手工摆位太慢，所以需要脚本化随机化。

## 设计约束（都是踩过的坑）
1. **先归位再随机化**
   任务执行完物体可能被移动过（拿到手里、插进孔里）。直接对「当前位置」加
   随机偏移会把物体甩到桌子上或穿透架子。所以顺序必须是：
     ① 物体（含挂在夹爪上的）回到**采集开始时**的位姿
     ② 再叠加随机偏移
2. **只随机化配置里声明的轴与范围**
   试管任务只需 xy（物体在桌面上，z 由桌面决定）。z 也随机会让物体悬空或
   陷进桌子。范围一律由配置给，代码里不写死。
3. **不假设物体一定自由**
   「试管架 + 试管」是两个刚体，且试管可能正被夹爪握着。随机化试管架时必须
   把试管一起搬（或先把试管归位到架上），否则试管留在原地、穿模。

## 配置格式（task yaml 的 ``randomize:`` 段）

```yaml
randomize:
  seed: 0                    # 0 = 每次用时间戳
  settle_steps: 30           # 设完位姿后让物理跑多少步（稳定）
  objects:
    - prim: /World/room/scene/test_tube_rack
      restore_from: /World/room/scene/test_tube_rack   # 省略 = 用 initial_poses
      axes: {x: 0.04, y: 0.03}   # ±范围（米）。未列出 = 不动
    - prim: /World/room/scene/test_tube
      follow: /World/room/scene/test_tube_rack         # 跟随另一个物体一起搬
      axes: {}
```

``follow`` 的语义：该物体不单独随机化，而是**跟着 ``follow`` 指定的物体
平移相同的偏移量**（保持相对布局）。适合「架子上插着试管」这种组合。

## 为什么需要「记录初始位姿」
Isaac 里没有可靠的「USD 原始位姿」读取路径（newton kit 关掉了
PhysX→USD 同步，Xform 读到的可能是 stale 值——见 DEBUG_LOG #1）。
所以策略是：**采集开始时（第一轮之前）抓一次位姿存进内存/meta**，
之后每轮都从这份快照恢复再随机化。
"""

from __future__ import annotations

import json
import time
from pathlib import Path
from typing import Any

import numpy as np

_LOG_PREFIX = "[randomize]"

#: 存初始位姿的文件名（放在会话/任务数据目录下，跨进程可复用）
SNAPSHOT_NAME = "scene_initial_poses.json"


def _log(msg: str) -> None:
    print(f"{_LOG_PREFIX} {msg}", flush=True)


# ------------------------------------------------------------------ 位姿读写


def _rigid(prim_path: str):
    """创建 RigidPrim（物体必须是刚体才有 get/set_world_poses）。"""
    from isaacsim.core.experimental.prims import RigidPrim

    return RigidPrim(prim_path)


def _to_np(v) -> np.ndarray:
    a = np.asarray(v, dtype=np.float64)
    return a.reshape(a.shape[-1]) if a.ndim else a


def get_pose(prim_path: str) -> tuple[np.ndarray, np.ndarray] | None:
    """读刚体世界位姿 → (pos(3), quat_wxyz(4))；失败返回 None。"""
    try:
        rp = _rigid(prim_path)
        p, q = rp.get_world_poses()
        pos = _to_np(p).reshape(-1)[:3]
        quat = _to_np(q).reshape(-1)[:4]
        if not (np.all(np.isfinite(pos)) and np.all(np.isfinite(quat))):
            return None
        return pos, quat
    except Exception as exc:  # noqa: BLE001
        _log(f"WARN 读位姿失败 {prim_path}: {exc}")
        return None


def set_pose(prim_path: str, pos: np.ndarray, quat: np.ndarray) -> bool:
    """写刚体世界位姿。

    **注意**：对 ``kinematic=True`` 的刚体无效；对 ``kinematic=False`` 的刚体
    会与物理求解器竞争（可能在后续步被改写）。所以写完之后必须让物理
    settle 若干步再读回验证——见 :func:`randomize_once` 的验证环节。
    """
    try:
        rp = _rigid(prim_path)
        rp.set_world_poses(
            positions=np.asarray(pos, dtype=np.float64).reshape(1, 3),
            orientations=np.asarray(quat, dtype=np.float64).reshape(1, 4),
        )
        return True
    except Exception as exc:  # noqa: BLE001
        _log(f"WARN 写位姿失败 {prim_path}: {exc}")
        return False


# ------------------------------------------------------------------ 配置解析


def load_config(task_config: Any) -> dict | None:
    """从任务配置里取 ``randomize`` 段。没有就返回 None（不做随机化）。"""
    if task_config is None:
        return None
    cfg = getattr(task_config, "randomize", None)
    if cfg is None and isinstance(task_config, dict):
        cfg = task_config.get("randomize")
    if not cfg:
        return None
    return dict(cfg)


def _axes_of(obj: dict) -> dict[str, float]:
    raw = obj.get("axes") or {}
    out: dict[str, float] = {}
    for ax in ("x", "y", "z"):
        if ax in raw:
            try:
                v = float(raw[ax])
            except (TypeError, ValueError):
                continue
            if v > 0:
                out[ax] = v
    return out


# ------------------------------------------------------------------ 快照


def snapshot_path(data_dir: str | Path) -> Path:
    return Path(data_dir) / SNAPSHOT_NAME


def save_snapshot(cfg: dict, data_dir: str | Path) -> dict[str, Any]:
    """记录所有待随机化物体的**当前**世界位姿（第一轮开始前调用一次）。"""
    snap: dict[str, Any] = {"taken_at": time.time(), "objects": {}}
    for obj in (cfg.get("objects") or []):
        prim = str(obj.get("prim") or "")
        if not prim:
            continue
        got = get_pose(prim)
        if got is None:
            snap["objects"][prim] = None
            continue
        pos, quat = got
        snap["objects"][prim] = {
            "pos": [float(x) for x in pos],
            "quat": [float(x) for x in quat],
        }
        _log(f"快照 {prim}: pos={np.round(pos, 4).tolist()}")
    p = snapshot_path(data_dir)
    try:
        p.parent.mkdir(parents=True, exist_ok=True)
        p.write_text(json.dumps(snap, indent=2, ensure_ascii=False), encoding="utf-8")
        _log(f"已保存初始位姿 → {p}")
    except OSError as exc:
        _log(f"WARN 保存快照失败: {exc}")
    return snap


def load_snapshot(data_dir: str | Path) -> dict | None:
    p = snapshot_path(data_dir)
    if not p.is_file():
        return None
    try:
        return json.loads(p.read_text(encoding="utf-8"))
    except Exception as exc:  # noqa: BLE001
        _log(f"WARN 读快照失败: {exc}")
        return None


# ------------------------------------------------------------------ 主流程


def restore_and_randomize(
    cfg: dict,
    *,
    data_dir: str | Path,
    rng: np.random.Generator | None = None,
    settle_fn=None,
) -> dict:
    """把物体恢复到初始位姿，再叠加随机偏移。

    Args:
        cfg: ``randomize`` 配置段（见模块 docstring）。
        data_dir: 存初始位姿快照的目录。
        rng: 随机源；None 时按 ``cfg['seed']`` 或时间戳创建。
        settle_fn: 可选，``settle_fn(n_steps)`` —— 让物理跑 n 步。
            不给就跳过（调用方负责推进）。

    Returns:
        ``{"ok": bool, "applied": [...], "warnings": [...], "seed": int}``
    """
    if rng is None:
        seed = int(cfg.get("seed") or 0) or int(time.time()) & 0x7FFFFFFF
        rng = np.random.default_rng(seed)
    else:
        seed = int(getattr(rng, "_fasim_seed", -1))

    snap = load_snapshot(data_dir)
    warnings: list[str] = []
    applied: list[dict] = []
    if snap is None:
        msg = f"没有初始位姿快照（{snapshot_path(data_dir)}）—— 先调 save_snapshot"
        _log(f"ERROR {msg}")
        return {"ok": False, "applied": [], "warnings": [msg], "seed": seed}

    objects = cfg.get("objects") or []

    # ---- 第一遍：把每个物体搬回初始位姿 ----
    # 先全部归位再做偏移，避免"A 挪了、B 还用旧相对位置去算"的错位。
    offsets: dict[str, np.ndarray] = {}
    for obj in objects:
        prim = str(obj.get("prim") or "")
        if not prim:
            continue
        rec = (snap.get("objects") or {}).get(prim)
        if not rec:
            warnings.append(f"{prim}: 快照里没有记录，跳过")
            continue
        axes = _axes_of(obj)
        off = np.zeros(3, dtype=np.float64)
        for i, ax in enumerate(("x", "y", "z")):
            if ax in axes:
                off[i] = float(rng.uniform(-axes[ax], axes[ax]))
        offsets[prim] = off
        target = np.asarray(rec["pos"], dtype=np.float64) + off
        quat = np.asarray(rec["quat"], dtype=np.float64)
        if set_pose(prim, target, quat):
            applied.append({
                "prim": prim,
                "offset": [round(float(v), 4) for v in off],
                "target": [round(float(v), 4) for v in target],
            })

    # ---- 第二遍：follow 关系的物体，跟着目标平移相同偏移 ----
    for obj in objects:
        prim = str(obj.get("prim") or "")
        src = str(obj.get("follow") or "")
        if not prim or not src:
            continue
        rec = (snap.get("objects") or {}).get(prim)
        if not rec:
            continue
        off = offsets.get(src)
        if off is None:
            warnings.append(f"{prim}: follow={src} 没有随机偏移，按原位不动")
            off = np.zeros(3)
        target = np.asarray(rec["pos"], dtype=np.float64) + off
        quat = np.asarray(rec["quat"], dtype=np.float64)
        if set_pose(prim, target, quat):
            applied.append({
                "prim": prim,
                "follow": src,
                "offset": [round(float(v), 4) for v in off],
            })

    # ---- 让物理稳定 ----
    n_settle = int(cfg.get("settle_steps", 0) or 0)
    if n_settle > 0 and settle_fn is not None:
        try:
            settle_fn(n_settle)
        except Exception as exc:  # noqa: BLE001
            warnings.append(f"settle 失败: {exc}")

    # ---- 验证：读回位姿，检查是否真的落到了目标附近 ----
    # set_world_poses 对 dynamic 刚体可能被物理改写（历史踩过），
    # 所以这里必须读回确认，不能假定成功。
    tol = float(cfg.get("verify_tol_m", 0.05) or 0.05)
    for a in applied:
        want = a.get("target")
        if not want:
            continue
        got = get_pose(a["prim"])
        if got is None:
            warnings.append(f"{a['prim']}: 读回失败，无法验证")
            continue
        err = float(np.linalg.norm(np.asarray(got[0]) - np.asarray(want)))
        a["verify_err_m"] = round(err, 4)
        if err > tol:
            warnings.append(
                f"{a['prim']}: 实际位姿与目标差 {err*1000:.0f}mm（> {tol*1000:.0f}mm）"
                " —— 物体可能被物理改写或与其它物体碰撞"
            )

    ok = not any("差" in w or "失败" in w for w in warnings)
    _log(f"随机化完成: {len(applied)} 个物体, seed={seed}, "
         f"warnings={len(warnings)}")
    for w in warnings:
        _log(f"  WARN {w}")
    return {"ok": ok, "applied": applied, "warnings": warnings, "seed": seed}
