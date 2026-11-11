"""Configuration loading: ``configs/_default.yaml`` + ``configs/<robot>/{robot,tasks}``.

Mirrors the repo convention that every robot gets its own directory and the
top-level layer is plain YAML.
"""

from __future__ import annotations

from pathlib import Path
from typing import Any

import yaml

from .types import CameraSpec, RobotSpec, TaskSpec

CONFIG_ROOT = Path(__file__).resolve().parents[3] / "configs"
"""``<ext>/configs`` — this module lives at ``<ext>/fasim/replicator/sdg``."""

REPO_ROOT = Path(__file__).resolve().parents[5]
"""FaSim-Isaac repo root.

``__file__`` = ``<repo>/exts/<ext>/fasim/replicator/sdg/config_io.py``
    parents[0] .. /sdg      parents[1] .. /replicator  parents[2] .. /fasim
    parents[3] = <ext>      parents[4] = <repo>/exts   parents[5] = <repo>
"""


class SdgConfigError(RuntimeError):
    """Raised when a config file is missing or malformed."""


def _read_yaml(path: Path) -> dict[str, Any]:
    if not path.is_file():
        raise SdgConfigError(f"config not found: {path}")
    try:
        data = yaml.safe_load(path.read_text(encoding="utf-8"))
    except yaml.YAMLError as exc:
        raise SdgConfigError(f"invalid YAML in {path}: {exc}") from exc
    if not isinstance(data, dict):
        raise SdgConfigError(f"{path} must contain a mapping at top level")
    return data


def load_default_config(config_root: Path | None = None) -> dict[str, Any]:
    root = config_root or CONFIG_ROOT
    return _read_yaml(root / "_default.yaml")


def _robot_dir(robot_key: str, config_root: Path) -> Path:
    path = config_root / robot_key
    if not path.is_dir():
        available = sorted(p.name for p in config_root.iterdir() if p.is_dir() and not p.name.startswith("_"))
        raise SdgConfigError(f"robot {robot_key!r} has no config directory under {config_root}; available: {available}")
    return path


def load_robot_config(robot_key: str, config_root: Path | None = None) -> RobotSpec:
    root = config_root or CONFIG_ROOT
    raw = _read_yaml(_robot_dir(robot_key, root) / "robot.yaml")
    try:
        return RobotSpec(
            robot_key=str(raw.get("robot_key") or robot_key),
            articulation_prim=str(raw["articulation_prim"]),
            base_prim=str(raw["base_prim"]),
            joint_state_topic=str(raw.get("joint_state_topic") or "/joint_states"),
            ee_prims={str(k): str(v) for k, v in (raw.get("ee_prims") or {}).items()},
            ee_links={
                str(k): {
                    "link": str(v.get("link") or ""),
                    "offset": [float(x) for x in (v.get("offset") or (0.0, 0.0, 0.0))],
                }
                for k, v in (raw.get("ee_links") or {}).items()
            },
            gripper_joints=tuple(str(j) for j in (raw.get("gripper_joints") or ())),
            expected_dof_count=raw.get("expected_dof_count"),
        )
    except KeyError as exc:
        raise SdgConfigError(f"{robot_key}/robot.yaml missing required key {exc}") from exc


def _parse_camera(raw: dict[str, Any]) -> CameraSpec:
    name = str(raw.get("name") or "")
    if not name:
        raise SdgConfigError("camera entry requires 'name'")
    res = raw.get("resolution") or [640, 480]
    if not (isinstance(res, (list, tuple)) and len(res) == 2):
        raise SdgConfigError(f"camera {name!r}: resolution must be [w, h]")
    annotators = raw.get("annotators")
    if annotators is None:
        annotators = ["rgb", "camera_params"]
    return CameraSpec(
        name=name,
        prim_path=(str(raw["prim"]) if raw.get("prim") else None),
        create=raw.get("create"),
        resolution=(int(res[0]), int(res[1])),
        annotators=tuple(str(a) for a in annotators),
        depth=bool(raw.get("depth", True)),
    )


def load_task_config(
    robot_key: str,
    task_key: str,
    config_root: Path | None = None,
    output_override: Path | None = None,
) -> TaskSpec:
    root = config_root or CONFIG_ROOT
    robot_dir = _robot_dir(robot_key, root)
    raw = _read_yaml(robot_dir / "tasks" / f"{task_key}.yaml")
    default = load_default_config(root)

    record = {**(default.get("record") or {}), **(raw.get("record") or {})}
    output = raw.get("output") or {}
    out_dir = output_override or Path(str(output.get("dir") or default["output"]["dir"]))

    scene = Path(str(raw["scene_usda"]))
    if not scene.is_absolute():
        # scene paths are repo-relative; resolve against the FaSim-Isaac root
        scene = REPO_ROOT / scene

    try:
        return TaskSpec(
            task_key=str(raw.get("task_key") or task_key),
            scene_usda=scene,
            tracked_prims={str(k): str(v) for k, v in (raw.get("tracked_prims") or {}).items()},
            cameras=tuple(_parse_camera(c) for c in (raw.get("cameras") or ())),
            output_dir=out_dir / robot_key / str(raw.get("task_key") or task_key),
            capture_fps=int(record.get("capture_fps", 30)),
            physics_dt=float(record.get("physics_dt", 0.005)),
            max_frames=int(record.get("max_frames", 2000)),
            variant_selection={str(k): str(v) for k, v in (raw.get("variant_selection") or {}).items()},
        )
    except KeyError as exc:
        raise SdgConfigError(f"{robot_key}/tasks/{task_key}.yaml missing required key {exc}") from exc
