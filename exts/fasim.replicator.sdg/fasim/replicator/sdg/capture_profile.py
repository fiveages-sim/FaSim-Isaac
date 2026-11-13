"""采集参数一致性校验 —— 批量合并前的守门人。

## 为什么需要
批量导出会把一个任务下的多个 run 合成一个 LeRobot 数据集（多个 episode）。
但只有**采集参数一致**时这样才能成立：

- 关节数不同（换了机器人/夹爪）→ 一个数据集装不下两种 state 维度
- 相机不同（少了腕部/改了分辨率）→ feature shape 冲突
- ``every`` 不同 → 每帧时长不同，时间轴混在一起会让策略学到错误的动作速度
- ``physics_hz`` 不同 → 同上，更隐蔽（5ms 还是 10ms 差别巨大）

这类错误在导出时才爆出来（lerobot 报 shape 不匹配），或者更糟 ——
静默写成功但数据语义混乱，训出来的策略无法解释。所以在**合并前**就拦住。

## 设计
``fingerprint(session)`` 从 ``session.json`` 抽出一组「影响数据格式与语义」
的字段，算成 dict。``compare(sessions)`` 比对多个会话的指纹，返回差异。

**只比对这些字段**（其余如时间戳、帧数、路径天然不同，不该比）。

## 用法
```python
from fasim.replicator.sdg import capture_profile as cp

rep = cp.compare(sessions)
if not rep.ok:
    print(cp.format_report(rep))
    # 拒绝合并
```
"""

from __future__ import annotations

import json
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any


#: 决定 LeRobot feature 定义（shape / names / key 集合）的字段。
#: 不一致时数据集**装不下**所有帧 —— 必须拒绝合并。
CRITICAL = (
    "joint_count",
    "joint_names",
    "cameras",
    "what",
    "robot_key",
    "task_key",
    "ee_captured",
    "objects_captured",
)

#: 不一致会产生「时间轴错乱」而非「直接报错」的字段。
#: 危险在于**静默成功** —— 所以也按 critical 处理。
TIMING = (
    "every",
    "physics_hz_setting",
)

#: 只在报告里提示、但**不阻止**合并的字段。
#: ``preset`` 名字不同没关系 —— 只要实际参数（上面的 CRITICAL/TIMING）
#: 一致就能合；``frames`` 天然不同（每条轨迹时长不一样）。
INFO_ONLY = ("frames", "preset")


def _load_session_json(session: Path) -> dict[str, Any]:
    p = session / "session.json"
    if not p.is_file():
        return {}
    try:
        return json.loads(p.read_text(encoding="utf-8"))
    except Exception:  # noqa: BLE001
        return {}


def _probe_joint_info(session: Path) -> tuple[int, tuple[str, ...]]:
    """从帧文件里读关节数与名字（session.json 不一定记了）。"""
    frames_dir = session / "frames"
    if not frames_dir.is_dir():
        return 0, ()
    files = sorted(frames_dir.glob("frame_*.npz"))
    if not files:
        return 0, ()
    import numpy as np

    try:
        with np.load(files[0], allow_pickle=False) as z:
            q = np.asarray(z["joint_pos"]).reshape(-1) if "joint_pos" in z.files else None
            names = (tuple(str(x) for x in z["joint_names"])
                     if "joint_names" in z.files else ())
            return (int(q.size) if q is not None else 0), names
    except Exception:  # noqa: BLE001
        return 0, ()


def fingerprint(session: Path) -> dict[str, Any]:
    """抽出一组「决定数据能否合并」的参数（可直接比较）。"""
    session = Path(session)
    meta = _load_session_json(session)

    n_joints, joint_names = _probe_joint_info(session)
    if not n_joints:
        n_joints = int(meta.get("joint_count") or 0)
    if not joint_names:
        joint_names = tuple(meta.get("joint_names") or ())

    # 相机：名字 + 分辨率（顺序不敏感）
    cams = []
    for c in (meta.get("cameras") or []):
        if isinstance(c, dict):
            size = c.get("size") or c.get("resolution") or ()
            try:
                size = tuple(int(x) for x in size)
            except (TypeError, ValueError):
                size = ()
            cams.append((str(c.get("name") or "?"), size))

    hz = meta.get("physics_hz_setting")
    if hz is None:
        pdt = float(meta.get("physics_dt", 0) or 0)
        hz = (1.0 / pdt) if pdt > 0 else None

    return {
        "joint_count": n_joints,
        "joint_names": tuple(joint_names),
        "cameras": tuple(sorted(cams)),
        "what": tuple(sorted(meta.get("what") or ())),
        "robot_key": str(meta.get("robot_key") or ""),
        "task_key": str(meta.get("task_key") or ""),
        "every": int(meta.get("every") or 0),
        "physics_hz_setting": (round(float(hz), 3) if hz else None),
        "ee_captured": tuple(sorted(meta.get("ee_captured") or ())),
        "objects_captured": tuple(sorted(meta.get("objects_captured") or ())),
        "capture_mode": str(meta.get("capture_mode") or "physics"),
        "preset": str(meta.get("preset") or ""),
        "frames": int(meta.get("frames_saved") or 0),
    }


@dataclass
class Mismatch:
    """一个字段在多个会话间的差异。"""

    key: str
    severity: str            # "critical" | "timing" | "info"
    values: dict[str, str]   # session 名 → 值（字符串化）


@dataclass
class Report:
    sessions: list[str] = field(default_factory=list)
    mismatches: list[Mismatch] = field(default_factory=list)
    fingerprints: dict[str, dict[str, Any]] = field(default_factory=dict)

    @property
    def critical(self) -> list[Mismatch]:
        return [m for m in self.mismatches if m.severity in ("critical", "timing")]

    @property
    def ok(self) -> bool:
        return not self.critical


def _fmt(v: Any) -> str:
    # 相机：((name, (w, h)), ...)
    if isinstance(v, tuple) and v and isinstance(v[0], tuple):
        parts = []
        for item in v:
            name, size = item[0], (item[1] if len(item) > 1 else ())
            if isinstance(size, tuple) and len(size) == 2:
                parts.append(f"{name}@{size[0]}x{size[1]}")
            else:
                parts.append(str(name))
        return "[" + ", ".join(parts) + "]"
    if isinstance(v, tuple):
        return "(" + ",".join(str(x) for x in v) + ")"
    return str(v)


def compare(sessions: list[Path]) -> Report:
    """比对多个会话的采集参数。``.ok`` 为 False 表示不能合并。"""
    fps: dict[str, dict[str, Any]] = {}
    for s in sessions:
        fps[Path(s).name] = fingerprint(Path(s))

    rep = Report(sessions=[str(s) for s in sessions], fingerprints=fps)
    if len(fps) < 2:
        return rep

    all_keys: set[str] = set()
    for f in fps.values():
        all_keys |= set(f)
    ordered = ([k for k in CRITICAL if k in all_keys]
               + [k for k in TIMING if k in all_keys]
               + sorted(all_keys - set(CRITICAL) - set(TIMING)))

    for k in ordered:
        vals = {name: f.get(k) for name, f in fps.items()}
        if len({_fmt(v) for v in vals.values()}) <= 1:
            continue
        sev = ("critical" if k in CRITICAL
               else "timing" if k in TIMING else "info")
        rep.mismatches.append(Mismatch(
            key=k, severity=sev,
            values={n: _fmt(v) for n, v in vals.items()},
        ))
    return rep


def format_report(rep: Report, *, max_sessions: int = 6) -> str:
    """把 Report 排成人能读的文本。"""
    if rep.ok and not rep.mismatches:
        return f"采集参数一致（{len(rep.sessions)} 个会话）"

    lines: list[str] = [f"采集参数检查：{len(rep.sessions)} 个会话"]
    lines.append("  ✓ 关键参数一致（可以合并）" if rep.ok
                 else "  ✗ 关键参数不一致 —— 不能合并成一个数据集")

    tag = {"critical": "必须一致", "timing": "时间轴", "info": "建议一致"}
    for m in rep.mismatches:
        mark = "✗" if m.severity in ("critical", "timing") else "·"
        lines.append(f"  {mark} {m.key}  [{tag[m.severity]}]")
        for i, (name, val) in enumerate(m.values.items()):
            if i >= max_sessions:
                lines.append(f"      … 还有 {len(m.values) - max_sessions} 个会话")
                break
            shown = val if len(val) <= 68 else val[:65] + "…"
            lines.append(f"      {name:24s} {shown}")

    if not rep.ok:
        lines.extend([
            "",
            "  怎么办：",
            "    1) 用同一套采集参数重采 —— 见 configs/<robot>/tasks/<task>.yaml",
            "       或菜单里选「采集预设」（自动带出全部参数）",
            "    2) 或者只导出参数一致的子集：给单个会话路径而不是任务目录",
        ])
    return "\n".join(lines)


def suggest_every(physics_hz: float | None, target_fps: float = 20.0) -> int:
    """按目标帧率反推 ``every``（物理步频已知时才有意义）。"""
    if not physics_hz or physics_hz <= 0:
        return 10
    return max(1, int(round(physics_hz / max(target_fps, 1e-6))))
