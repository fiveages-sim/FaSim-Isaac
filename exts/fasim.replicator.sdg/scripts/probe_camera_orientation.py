"""Ground truth for camera orientation: read world transforms straight from USD.

cameraViewTransform in Replicator's camera_params is easy to transpose by mistake,
so this reads UsdGeom.XformCache directly and reports the true look direction
(-Z in USD camera convention).
"""
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from isaacsim import SimulationApp

app = SimulationApp(launch_config={"headless": True})
try:
    import numpy as np
    import omni.timeline
    import omni.usd
    from pxr import Gf, Usd, UsdGeom

    from fasim.replicator.sdg.config_io import load_robot_config, load_task_config
    from fasim.replicator.sdg.recorder import _apply_variants

    robot = load_robot_config("unitree_g1d")
    task = load_task_config("unitree_g1d", "sinopec_test_tube")

    ctx = omni.usd.get_context()
    ctx.open_stage(str(task.scene_usda))
    for _ in range(40):
        app.update()
    _apply_variants(robot.articulation_prim, task.variant_selection)
    for _ in range(60):
        app.update()

    # let physics settle WITHOUT driving joints
    tl = omni.timeline.get_timeline_interface()
    tl.play()
    for _ in range(120):
        app.update()

    stage = ctx.get_stage()
    cache = UsdGeom.XformCache(Usd.TimeCode.Default())

    print("\n=== robot base (world) ===")
    bp = stage.GetPrimAtPath(robot.base_prim)
    m = cache.GetLocalToWorldTransform(bp)
    print(f"  {robot.base_prim} t={np.round(list(m.ExtractTranslation()),4)}")

    print("\n=== cameras: USD world transform (ground truth) ===")
    hdr = f"{'camera':12s} {'world position':34s} {'look (-Z)':28s} {'up (+Y)':24s}"
    print(hdr)
    print("-" * len(hdr))
    for cam in task.cameras:
        prim = stage.GetPrimAtPath(cam.prim_path)
        if not prim or not prim.IsValid():
            print(f"{cam.name:12s} MISSING")
            continue
        m = cache.GetLocalToWorldTransform(prim)
        R = np.array([[m[r][c] for c in range(3)] for r in range(3)])
        t = np.array([m[3][0], m[3][1], m[3][2]])
        look = -R[:, 2]      # USD camera looks along -Z
        up = R[:, 1]
        print(f"{cam.name:12s} {str(np.round(t,4)):34s} {str(np.round(look,3)):28s} {str(np.round(up,3)):24s}")

    print("\n=== sanity: what SHOULD the look direction be? ===")
    print("  robot faces world -X; a forward-looking head cam -> look ≈ (-0.7, 0, -0.7) (forward+down)")
    print("  wrist cams mounted at the finger side should share that forward hemisphere")

    # also verify camera_params round-trip so we know the layout
    print("\n=== camera_params.cameraViewTransform layout check (head) ===")
    import json
    cams = next(c for c in task.cameras if c.name == "head")
    # write one frame is heavyweight; instead compare matrix from USD above with
    # the values observed earlier: if transpose matches, column 3 == translation
    print("  (skipped: needs a rendered frame; USD transform above is authoritative)")
finally:
    app.close()
