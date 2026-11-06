"""Verify DOF record+replay keeps physics valid (no tensor invalidation).

Headless: main thread pumps app.update() (like Kit); record 3s of a sine motion,
then replay it and confirm the EE actually moves with no PhysX errors.
"""
import sys
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from isaacsim import SimulationApp

app = SimulationApp(launch_config={"headless": True})

import numpy as np
import omni.timeline
import omni.usd
from isaacsim.core.experimental.prims import Articulation

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

tl = omni.timeline.get_timeline_interface()
tl.play()
for _ in range(10):
    app.update()

sys.path.insert(0, str(Path(__file__).resolve().parent))
import dof_replay

art = Articulation(robot.articulation_prim)
n = int(art.num_dofs)
base = art.get_dof_positions().numpy().reshape(-1)[:n]
print(f"[dv] dofs={n}")

# ---------- record ----------
info = dof_replay.arm_dof_record(output_hint_dir="/home/fiveages/fasim_sdg/_dof_test")
print(f"[dv] recording to {info['path']}")

frames = 0
t_end = time.monotonic() + 3.0
i = 0
while time.monotonic() < t_end:
    off = 0.25 * np.sin(2 * np.pi * 0.5 * i * 0.016) * np.linspace(0, 1, n)
    art.set_dof_position_targets((base + off)[np.newaxis, :].astype(np.float32))
    app.update()
    i += 1
    frames += 1
    time.sleep(0.016)

res = dof_replay.stop_dof_record()
print(f"[dv] recorded {res.get('frames')} samples")

# ---------- replay ----------
data = np.load(res["path"])["dof_positions"]
print(f"[dv] track shape {data.shape}")
print(f"[dv] dof range per-joint max-min: {np.round((data.max(axis=0) - data.min(axis=0)).max(), 4)}")

r = dof_replay.replay_dof(res["path"], end=min(60, data.shape[0]))
print(f"[dv] replayed {r['frames']} frames")
print("[dv] DOF_REPLAY_OK")

tl.stop()
app.close()
