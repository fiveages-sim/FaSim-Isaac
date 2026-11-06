"""Shared bootstrap for the SDG shell entrypoints."""

from __future__ import annotations

import os
import sys
from pathlib import Path

EXT_ROOT = Path(__file__).resolve().parents[3]


def python_bin() -> Path:
    """Isaac's bundled interpreter (has pxr / omni.*)."""
    isaac = os.environ.get("ISAACSIM_DIR", str(Path.home() / "isaacsim"))
    return Path(isaac) / "python.sh"


def build_env() -> dict[str, str]:
    env = dict(os.environ)
    # Make `fasim.replicator.sdg` importable without installing the extension.
    paths = [str(EXT_ROOT)]
    if env.get("PYTHONPATH"):
        paths.append(env["PYTHONPATH"])
    env["PYTHONPATH"] = os.pathsep.join(paths)
    return env
