#!/usr/bin/scene python3
"""Diagnose CR5 in-process ros2_control setup without the OmniGraph node."""

from __future__ import annotations

import os
import sys
import traceback

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
EMIT = os.path.join(os.path.dirname(__file__), "emit_control.py")


def main() -> int:
    from isaacsim import SimulationApp

    simulation_app = SimulationApp({"renderer": "RayTracedLighting", "headless": True})

    import carb
    import isaacsim.core.experimental.utils.app as app_utils
    import omni.usd
    from isaacsim.core.simulation_manager import SimulationManager
    from pxr import UsdPhysics

    app_utils.enable_extension("isaacsim.ros2.core")
    app_utils.enable_extension("isaacsim.ros2.bridge")
    app_utils.enable_extension("isaacsim.ros2.control")
    app_utils.enable_extension("fasim.ros2.nodes")
    simulation_app.update()

    from isaacsim.ros2.control import Ros2ControlManager
    from isaacsim.ros2.control.urdf_synth import build_full_urdf

    omni.usd.get_context().open_stage(STAGE)
    for _ in range(60):
        simulation_app.update()

    stage = omni.usd.get_context().get_stage()
    roots = [p.GetPath().pathString for p in stage.Traverse() if p.HasAPI(UsdPhysics.ArticulationRootAPI)]
    print("articulation_roots:", roots)

    SimulationManager.setup_simulation(dt=1.0 / 60.0, device="cpu")
    app_utils.play()
    for _ in range(30):
        simulation_app.update()
    app_utils.pause()
    for _ in range(5):
        simulation_app.update()

    try:
        urdf = build_full_urdf(stage, "/World/CR5")
        out = "/tmp/cr5_synth.urdf"
        with open(out, "w", encoding="utf-8") as handle:
            handle.write(urdf)
        print("synth_urdf:", out, "bytes", len(urdf))
        print("has_ros2_control", "<ros2_control" in urdf)
        print("plugin", "isaacsim_ros2_control/IsaacSimSystem" in urdf)
        # joint names
        import re

        joints = re.findall(r'<joint name="([^"]+)"', urdf)
        print("urdf_joints_sample:", joints[:20], "count", len(joints))
        rc_joints = re.findall(r'<ros2_control[\s\S]*?</ros2_control>', urdf)
        print("ros2_control_blocks", len(rc_joints))
        if rc_joints:
            print(rc_joints[0][:1500])
    except Exception:
        traceback.print_exc()
        simulation_app.close()
        return 1

    import json
    import subprocess

    proc = subprocess.run(
        [sys.executable if False else "python3", EMIT, "--robot", "cr5", "--mode", "demo", "--enable-gripper", "false"],
        capture_output=True,
        text=True,
        check=False,
    )
    print("emit_stderr:\n", proc.stderr)
    print("emit_stdout:\n", proc.stdout)
    if proc.returncode != 0:
        simulation_app.close()
        return 2
    payload = json.loads([l for l in proc.stdout.splitlines() if l.strip()][-1])
    yaml_path = payload["yaml_path"]
    print("yaml_path", yaml_path)

    try:
        rc = Ros2ControlManager.setup(
            "/World/CR5",
            yaml_path,
            publish_robot_description=False,
            use_sim_time=True,
        )
        print("setup_rc", rc)
    except Exception as exc:
        print("setup_exc", type(exc), exc)
        traceback.print_exc()
        simulation_app.close()
        return 3

    app_utils.play()
    for _ in range(60):
        simulation_app.update()

    app_utils.stop()
    simulation_app.close()
    return 0


if __name__ == "__main__":
    sys.exit(main())
