"""Ensure ROS 2 shared libs resolve with workspace-over-underlay order."""

from __future__ import annotations

import os

import carb


def _unique_append(parts: list[str], path: str) -> None:
    if path and path not in parts and os.path.isdir(path):
        parts.append(path)


def ensure_isaac_ros2_library_path() -> str | None:
    """Rebuild ``LD_LIBRARY_PATH`` for in-process controller plugin dlopen.

    Order (first wins):

    1. ``isaacsim.ros2.core/<distro>/lib`` — Kit tinyxml2.so.9 / CM plugins
    2. Non-``/opt/ros`` AMENT prefix ``lib`` dirs — ros2_ws overlay (ocs2_*)
    3. Existing non-underlay entries already on ``LD_LIBRARY_PATH``
    4. ``/opt/ros/<distro>/lib`` (+ ``lib/x86_64-linux-gnu``) — tracetools,
       pinocchio, coal underlay

    Host workspaces often ship a newer libtinyxml2 while Isaac needs .so.9.
    Stripping all of ``/opt/ros`` (pink-safe vanilla launch) removes
    ``libtracetools.so`` and breaks ``ocs2_wbc_controller``. ``./run.sh`` keeps
    underlay on LD and disables pink; this helper still reorders so a stray
    apt ``libocs2_*.so`` cannot shadow the workspace build.
    """
    distro = os.environ.get("ROS_DISTRO", "").strip() or "jazzy"
    try:
        import omni.kit.app

        ext_manager = omni.kit.app.get_app().get_extension_manager()
        core_path = ext_manager.get_extension_path_by_module("isaacsim.ros2.core")
    except Exception as exc:
        carb.log_warn(f"fasim.ros2: cannot resolve isaacsim.ros2.core path: {exc}")
        core_path = None

    isaac_lib: str | None = None
    if core_path:
        isaac_lib = os.path.join(core_path, distro, "lib")
        if not os.path.isdir(isaac_lib):
            isaac_lib = None
            for candidate in ("jazzy", "humble"):
                alt = os.path.join(core_path, candidate, "lib")
                if os.path.isdir(alt):
                    isaac_lib = alt
                    break

    overlay: list[str] = []
    underlay: list[str] = []

    if isaac_lib:
        _unique_append(overlay, isaac_lib)

    for prefix in (os.environ.get("AMENT_PREFIX_PATH") or "").split(":"):
        prefix = prefix.strip().rstrip("/")
        if not prefix:
            continue
        lib = os.path.join(prefix, "lib")
        if prefix.startswith("/opt/ros"):
            _unique_append(underlay, lib)
            continue
        if prefix == (isaac_lib or "").rstrip("/"):
            continue
        _unique_append(overlay, lib)

    for entry in (os.environ.get("LD_LIBRARY_PATH") or "").split(":"):
        entry = entry.strip().rstrip("/")
        if not entry:
            continue
        if entry.startswith("/opt/ros"):
            _unique_append(underlay, entry)
        elif entry not in overlay:
            overlay.append(entry)

    for path in (
        f"/opt/ros/{distro}/lib",
        f"/opt/ros/{distro}/lib/x86_64-linux-gnu",
    ):
        _unique_append(underlay, path)

    new_path = ":".join(overlay + underlay)
    if new_path and new_path != os.environ.get("LD_LIBRARY_PATH", ""):
        os.environ["LD_LIBRARY_PATH"] = new_path
        carb.log_info(
            "fasim.ros2: LD_LIBRARY_PATH overlay-first "
            f"(isaac/ws then /opt/ros; {len(overlay)}+{len(underlay)} entries)"
        )
    return isaac_lib
