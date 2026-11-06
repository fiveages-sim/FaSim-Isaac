"""Probe the scene for configured camera prims without rendering (headless)."""
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from isaacsim import SimulationApp

app = SimulationApp(launch_config={"headless": True})
try:
    import omni.usd
    from fasim.replicator.sdg.config_io import load_robot_config, load_task_config
    from fasim.replicator.sdg.recorder import _apply_variants

    robot = load_robot_config("unitree_g1d")
    task = load_task_config("unitree_g1d", "sinopec_test_tube")

    ctx = omni.usd.get_context()
    ctx.open_stage(str(task.scene_usda))
    for _ in range(30):
        app.update()

    stage = ctx.get_stage()
    print(f"\nstage: {stage.GetRootLayer().identifier}")

    _apply_variants(robot.articulation_prim, task.variant_selection)
    for _ in range(60):
        app.update()

    print(f"\n{'camera':14s} {'exists':7s} prim")
    print("-" * 100)
    missing = []
    for cam in task.cameras:
        prim = stage.GetPrimAtPath(cam.prim_path) if cam.prim_path else None
        ok = bool(prim and prim.IsValid())
        if not ok:
            missing.append(cam)
        print(f"{cam.name:14s} {'YES' if ok else 'NO ':7s} {cam.prim_path}")
        if ok:
            print(f"{'':22s}type={prim.GetTypeName()}")

    print("\ntracked prims:")
    for label, path in task.tracked_prims.items():
        prim = stage.GetPrimAtPath(path)
        ok = bool(prim and prim.IsValid())
        print(f"  {label:18s} {'OK ' if ok else 'MISSING'} {path}")

    print("\narticulation:")
    p = stage.GetPrimAtPath(robot.articulation_prim)
    print(f"  {robot.articulation_prim} valid={bool(p and p.IsValid())}")

    if missing:
        print("\n=== MISSING CAMERAS (report to user) ===")
        for cam in missing:
            print(f"  {cam.name}: {cam.prim_path}")
            # suggest candidates by scanning for Camera prims
            cams = [str(pr.GetPath()) for pr in stage.Traverse()
                    if pr.GetTypeName() == "Camera"]
            print(f"    stage has {len(cams)} Camera prim(s):")
            for c in cams[:12]:
                print(f"      {c}")
finally:
    app.close()
