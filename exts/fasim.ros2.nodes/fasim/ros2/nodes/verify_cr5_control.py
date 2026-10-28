#!/usr/bin/scene python3
"""Headless smoke test: open CR5 scene/control.usda, Play, list active controllers."""

from __future__ import annotations

import os
import subprocess
import sys
import time

STAGE = os.path.abspath(
    os.path.join(
        os.path.dirname(__file__),
        "..",
        "..",
        "..",
        "..",
        "..",
        "robots",
        "manipulators",
        "Dobot",
        "CR5",
        "scene",
        "fiveages.usda",
    )
)


def _list_controllers(namespace: str = "") -> dict[str, str]:
    """Query controller states via CLI to avoid sharing Isaac's rclpy executor."""
    cm = f"/{namespace.strip('/')}/controller_manager" if namespace else "/controller_manager"
    proc = subprocess.run(
        ["ros2", "control", "list_controllers", "-c", cm],
        capture_output=True,
        text=True,
        check=False,
    )
    active: dict[str, str] = {}
    if proc.returncode != 0:
        return active
    for line in (proc.stdout or "").splitlines():
        parts = line.split()
        if len(parts) >= 2:
            # name type state  OR  name state
            name = parts[0]
            state = parts[-1]
            if state in ("active", "inactive", "unconfigured", "configured"):
                active[name] = state
    return active


def main() -> int:
    isaac_ros2_lib = os.path.join(
        os.environ.get("ISAACSIM_DIR", os.path.expanduser("~/isaacsim")),
        "exts",
        "isaacsim.ros2.core",
        os.environ.get("ROS_DISTRO", "jazzy"),
        "lib",
    )
    if os.path.isdir(isaac_ros2_lib):
        current = os.environ.get("LD_LIBRARY_PATH", "")
        if isaac_ros2_lib not in current.split(":"):
            os.environ["LD_LIBRARY_PATH"] = isaac_ros2_lib + ((":" + current) if current else "")

    from isaacsim import SimulationApp

    simulation_app = SimulationApp({"renderer": "RayTracedLighting", "headless": True})

    import carb
    import isaacsim.core.experimental.utils.app as app_utils
    import omni.usd
    from isaacsim.core.simulation_manager import SimulationManager

    app_utils.enable_extension("isaacsim.ros2.core")
    app_utils.enable_extension("isaacsim.ros2.bridge")
    app_utils.enable_extension("isaacsim.ros2.control")
    app_utils.enable_extension("fasim.ros2.nodes")
    simulation_app.update()

    from fasim.ros2.nodes.library_path import ensure_isaac_ros2_library_path

    ensure_isaac_ros2_library_path()

    if not os.path.isfile(STAGE):
        carb.log_error(f"stage missing: {STAGE}")
        simulation_app.close()
        return 1

    omni.usd.get_context().open_stage(STAGE)
    for _ in range(30):
        simulation_app.update()

    SimulationManager.setup_simulation(dt=1.0 / 60.0, device="cpu")
    app_utils.play()

    active: dict[str, str] = {}
    deadline = time.monotonic() + 180.0
    last_poll = 0.0
    while simulation_app.is_running() and time.monotonic() < deadline:
        simulation_app.update()
        now = time.monotonic()
        if now - last_poll >= 2.0:
            last_poll = now
            active = _list_controllers("")
            print("poll", active, flush=True)
            if (
                active.get("joint_state_broadcaster") == "active"
                and active.get("ocs2_arm_controller") == "active"
            ):
                break
        time.sleep(0.02)

    print("controllers:", active, flush=True)
    ok = (
        active.get("joint_state_broadcaster") == "active"
        and active.get("ocs2_arm_controller") == "active"
    )
    if ok:
        print("PASS", flush=True)
    elif active.get("joint_state_broadcaster") != "active":
        print("FAIL: joint_state_broadcaster not active", file=sys.stderr, flush=True)
    else:
        print("FAIL: ocs2_arm_controller not active", file=sys.stderr, flush=True)

    try:
        app_utils.stop()
    except Exception:
        pass
    try:
        simulation_app.close()
    except Exception:
        pass
    return 0 if ok else (2 if active.get("joint_state_broadcaster") != "active" else 3)


if __name__ == "__main__":
    sys.exit(main())
