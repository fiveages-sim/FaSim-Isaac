"""Replicator annotator registry with validation.

Only annotators that exist in the running Isaac build are exposed, so a config
typo surfaces as a clear error instead of a silent empty array.
"""

from __future__ import annotations

from typing import Any

#: annotator name -> (dtype description, channels, extra keys)
ANNOTATOR_REGISTRY: dict[str, dict[str, Any]] = {
    "rgb": {"dtype": "uint8", "shape": "HxWx4", "note": "RGBA, drop alpha for training"},
    "rgba": {"dtype": "uint8", "shape": "HxWx4", "note": "alias of rgb"},
    "distance_to_image_plane": {
        "dtype": "float32",
        "shape": "HxW",
        "note": "metres, per-pixel along camera ray; preferred over depth_to_image_plane",
    },
    "distance_to_camera": {"dtype": "float32", "shape": "HxW", "note": "metres, radial"},
    "camera_params": {
        "dtype": "dict",
        "shape": "-",
        "note": "intrinsics + world pose; required for calibration checks",
    },
    "semantic_segmentation": {
        "dtype": "uint8",
        "shape": "HxWx4",
        "extra": ["info/idToLabels"],
        "note": "needs Semantics labels authored on prims",
    },
    "instance_segmentation": {"dtype": "uint32", "shape": "HxW", "extra": ["info/idToLabels"]},
    "bounding_box_2d_tight": {"dtype": "struct", "shape": "-", "note": "per-object boxes"},
    "bounding_box_3d": {"dtype": "struct", "shape": "-", "note": "per-object 3D boxes"},
    "pointcloud": {"dtype": "float32", "shape": "Nx3", "note": "requires depth + camera_params"},
    "normals": {"dtype": "float32", "shape": "HxWx3"},
    "motion_vectors": {"dtype": "float32", "shape": "HxWx4"},
}


def resolve_annotators(names: tuple[str, ...] | list[str]) -> list[str]:
    """Validate requested annotator names against the registry."""
    unknown = [n for n in names if n not in ANNOTATOR_REGISTRY]
    if unknown:
        raise ValueError(
            f"unknown annotator(s) {unknown}; supported: {sorted(ANNOTATOR_REGISTRY)}"
        )
    return list(names)


def describe_annotators(names: tuple[str, ...] | list[str]) -> str:
    """Human-readable summary used by the CLI ``--describe`` flag."""
    lines = []
    for name in names:
        meta = ANNOTATOR_REGISTRY.get(name)
        if meta is None:
            lines.append(f"  {name:28s} UNKNOWN")
            continue
        extra = f" extra={meta['extra']}" if meta.get("extra") else ""
        lines.append(
            f"  {name:28s} {meta['dtype']:8s} {meta['shape']:8s}{extra}  {meta.get('note', '')}"
        )
    return "\n".join(lines)
