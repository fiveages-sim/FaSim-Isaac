---
name: fasim-ros2-control-launch
description: >-
  FaSim Isaac in-process ros2_control via OgnROS2ControlLaunch + emit_control
  (ControllerManager, synth URDF repair, arms_target_manager / VR frames).
  Use when wiring ROS=fiveages, debugging createArticulationView / tensors
  invalidated, duplicate URDF links, ATM VR teleop weirdness, G1/G1D/W2
  target_manager.yaml, or AssemblerFixedJoint snaps on Play.
---

# FaSim ROS2 Control Launch (Isaac in-process)

OmniGraph node `fasim.ros2.nodes.ROS2ControlLaunch` (`OgnROS2ControlLaunch.py`) +
`emit_control.py`: merge robot_common_launch YAML → Isaac in-process
ControllerManager → optional host `arms_target_manager` (ATM / VR).

Companion: `USDA-OCS2-PhysX-Mujoco` (asset Physics / OCS2 stack). This skill is
the **node + synth + ATM** playbook from G1/G1D bring-up.

## Quick map

| Piece | Path / role |
|-------|-------------|
| OGN | `exts/fasim.ros2.nodes/.../OgnROS2ControlLaunch.ogn` |
| Node | `.../OgnROS2ControlLaunch.py` (`_fix_synth_urdf`, ATM spawn) |
| Emit | `exts/fasim.ros2.nodes/.../emit_control.py` |
| Robot YAML | `<robot>_description/config/ros2_control/` + `config/ocs2/` |
| Env wiring | `robots/**/payloads/ROS/fiveages.usda` → Graph `ROS2ControlLaunch` |

## 1. `targetPrim` = ArticulationRoot PhysX can see

`createArticulationView(path)` / tensors match the prim that has
`PhysicsArticulationRootAPI` **and** a successfully parsed PhysX articulation.

| Pattern | ArtRoot on | Controllers target | When |
|---------|------------|--------------------|------|
| **W2** | `/FiveAges_W2/base_link` | same | `base_link` hosts native body joints |
| **G1D composite** | `/G1D` (robot Xform) | `/G1D` | Stub hub `/G1D/base_link` is Assembler-only — ArtRoot there → `Pattern did not match any articulations` |

Rules:

- Sole **active** ArtRoot on the whole robot; strip Chassis/Lift/Body/Arms importer roots.
  EE uses §2.1 (`root_joint active=false`), not `delete` ArtRoot on the gripper base.
- Floating hub (`/G1D/base_link`): `RigidBody` + real mass/inertia (tiny mass rattles); **not** ArtRoot on G1D-style composites.
- Env: pin `Physics=physx` so VariantSwitcher cannot flip to mujoco before CM configure.
- Do **not** change EE variants while Playing — deleted finger prims invalidate tensors.

## 2. AssemblerFixedJoint must match authored TF

PhysX warning `CreateJoint - disjointed body transforms` → snap → cascade
`simulationView invalidated` / MimicJoint “not part of an articulation”.

- **Chassis → hub**: `localPos0` = chassis `base_link` world offset relative to hub.
  G1D: chassis `base_link` at z=`0.1114` → `localPos0=(0,0,0.1114)` (same deck height as AGV). `localPos=0` with offset bodies will yank the whole stack.
- Match W2: chassis `xformOp:translate` and Assembler `localPos0` stay consistent.
- Lift → Chassis `AGV_link`: keep serial stack; `localPos0` = AGV-relative column mount (G1D ≈ z `0.5275`).

Caster/wheel USD `±inf` limits: clear in Chassis adapter (or synth sanitizer). Inf can break articulation parse.

## 2.1 EE / gripper articulation merge (Dex1, AG2F90, Revo, …)

PhysX **forbids nested ArticulationRoots**. `ROS2ControlLaunch` also requires
**exactly one** active `ArticulationRootAPI` under `targetPrim`. EE must merge
into the parent art; Mimic joints must end up **inside that same art**.

### Where the asset puts ArtRoot

| Pattern | ArtRoot prim | Examples | Standalone | Mounted under robot |
|---------|--------------|----------|------------|---------------------|
| **A (preferred)** | `Physics/root_joint` (FixedJoint) | AG2F90, Dex1 (after fix) | root_joint active | `root_joint (active=false)` → no live child root; fingers join parent via AssemblerFixedJoint |
| **B (legacy / bad for nest)** | gripper `*_base_link` RB | old Dex1 on `dex1_base_link` | works | Keeping it → `Nested articulation roots are not allowed` + CM “N ArtRoot prims”. Deleting it without merge → `PhysxMimicJointAPI … not part of an articulation` → tensors invalidate → **arms scatter** |

**Always author new grippers as pattern A.** If an asset is still B, move
`PhysicsArticulationRootAPI` (+ `NewtonArticulationRootAPI`) onto `root_joint`
and leave only `PhysicsRigidBodyAPI` / mass on the base link
(`grippers/Unitree/Dex1/payloads/Physics/physics.usda`).

### Mount adapter checklist

Proven in-tree (G1D Revo2, CR5 AG2F90, G1 Dex1):

```usda
over "Physics" {
    over "root_joint" (
        active = false
        # active=false alone can still leave a nested ArtRoot that PhysX
        # rejects → Mimic not in art → tensors invalidate → arms scatter.
        delete apiSchemas = ["PhysicsArticulationRootAPI", "NewtonArticulationRootAPI"]
    ) { }
}
over "Geometry" {
    over "dex1_base_link" {   # or gripper_base / hand_base
        def PhysicsFixedJoint "AssemblerFixedJoint" {
            # Match Revo2/CR5/G1: body0 = *_tcp (PhysX walks to parent wrist RB).
            rel physics:body0 = </Robot/.../wrist_yaw_link/right_tcp>
            rel physics:body1 = </Robot/.../tcp/Dex1/Geometry/dex1_base_link>
            bool physics:excludeFromArticulation = false
            # localPos0 = EE xform translate under tcp (Dex1 translate=0 → 0).
            point3f physics:localPos0 = (0, 0, 0)
            point3f physics:localPos1 = (0, 0, 0)
        }
    }
}
```

Rules:

1. **EE `root_joint`**: `active=false` **and** `delete` ArtRoot schemas (CR5/Revo2).
   Do **not** delete ArtRoot on a **standalone arm** `root_joint` that is the
   sole art (USDA-OCS2: drops child joints / `CreateJoint - no bodies`).
2. **`body0` = `*_tcp`** (same as Revo2/G1/CR5). tcp may be Xform-only; PhysX
   resolves to the parent wrist RB. Prefer this over inventing wrist+offset
   unless you have measured TF.
3. **`localPos0/localRot0` = EE translate/orient under tcp** (not double-count
   tcp's own wrist offset). Mismatch → disjointed snap → tensors invalidate.
4. **Never** leave a live ArtRoot under the robot Xform (dual Dex1 = 3 roots).
5. **Never** hot-swap EE / Physics while Playing — deleted finger prims invalidate tensors.
6. Parent `isaac:physics:robotJoints/Links` must include the composed EE path;
   Side left|right must retarget Dex1 `robotJoints` to `*_gripper_joint*`.
7. Env `PhysicsScene`: PhysX-only when running PhysX (no dual `NewtonSceneAPI`)
   — dual scene can delete Lift instanceable collisions → tensors invalidate.

### Failure cascade (G1D / Dex1)

```
Nested ArtRoot under /G1D
  → PhysX reject / CM "contains N ArticulationRootAPI prims"
Strip ArtRoot on base_link, weld fails or body0=tcp
  → PhysxMimicJointAPI not part of an articulation
  → simulationView invalidated / getDofPositions failed
  → arms lose hold → scatter from torso
```

Fix: pattern A on asset + mount `root_joint active=false` + wrist RB weld.

## 3. Synth URDF repair (`_fix_synth_urdf`)

Isaac USD→URDF uses **leaf prim names**. Composites collide:

- Hub + Chassis both export `base_link` → urdfdom: `link 'base_link' is not unique`
- Dual Dex1: duplicate `dex1_base_link` / fingers

Node already:

1. Inject missing EE kinematics / HW joints  
2. Bridge orphan Assembler FixedJoints  
3. `±inf` revolute → `continuous`  
4. **`_dedupe_urdf_duplicate_names`**: rename `name__N`, retarget parent/child (hub→chassis, left/right EE)

After Python changes: reload extension or restart Kit.

Debug dumps: `/tmp/fasim_synth_urdf_*.urdf`, `/tmp/arms_target_manager_*.yaml`.

## 4. ATM / VR frames (`target_manager.yaml`)

ATM is a **host** process (`use_sim_time=false`, nice=10). CM stays on sim time + `/clock`.

Each robot **must** ship `config/ocs2/target_manager.yaml`. Without it, defaults are W2-centric and break other robots:

| Default (ATM package) | Problem |
|-----------------------|---------|
| `enable_vr: false` | VR off |
| `vr_follow_frame: base_footprint` | Standing G1 has no footprint |
| `reference_link` → C++ `head_link2` | G1/G1D use `head_link` |

### Per-robot VR contract

| Robot | `vr_follow_frame` | `reference_link` / `head_link_name` | Notes |
|-------|-------------------|-------------------------------------|-------|
| G1 | `base_link` | `head_link` | No `base_footprint` |
| G1D | `base_footprint` | `head_link` | Wheeled; Isaac ROS TF publishes footprint |
| W2 | `base_footprint` | `head_link2` | Reference |

**Never** set `vr_follow_frame=world` on a mobile base — yaw twists hand offsets
(VRInputHandler: deltas in follow frame, publish in EE / WBC frame).

### full_body + WBC

- Info file from `ocs2_wbc_controller.info_file_name` (G1D: `mobile_base_tcp`, modelType 1 → parse forces world).
- `emit_control`: when WBC available, force **`marker_fixed_frame=world`** and **`control_base_frame=world`** (extras), so a wrong arms-only `task.info` (`baseFrame=torso_link`) cannot leave continuous targets chassis-offset.
- Keep `vr_follow_frame` on footprint/base in `target_manager.yaml`; only marker/control base go to world.
- `enable_wbc_head_tracking_marker`: emit sets true for WBC; yaml default false for non-full_body.

split_body / demo: use arm `task.info`; markers often stay on footprint/base_link.

## 5. Env / Graph checklist

```
- [ ] targetPrim = ArtRoot prim (G1D: /World/G1D; W2: /World/FiveAges_W2/base_link)
- [ ] mode = full_body | split_body | demo; robot = unitree_g1d / unitree_g1 / …
- [ ] enableArmsTargetManager; type empty → EE variant names map to ROS keys
- [ ] Physics=physx pinned; ROS=fiveages; /clock present if useSimTime
- [ ] target_manager.yaml present for that description package (src + install)
- [ ] Chassis/EE Assembler localPos matches USD TF; EE body0 is RigidBody
- [ ] After Play: no “link not unique”, no tensors invalidated, ATM logs correct frames
```

## 6. Common errors → fix

| Log / symptom | Fix |
|---------------|-----|
| `Pattern '…/base_link' did not match any articulations` | ArtRoot on robot Xform (composite); retarget controllers |
| `disjointed body transforms` / whole robot snaps | Fix Assembler `localPos0` to match link offset |
| MimicJoint / nested ArtRoot / Dex1 | ArtRoot on `root_joint`; mount `active=false`; weld to wrist RB |
| `link 'base_link' is not unique` | Need `_dedupe_urdf_duplicate_names` (reload node) |
| ATM VR “weird” / no teleop | Add/fix `target_manager.yaml` frames; enable_vr |
| Continuous track offset on mobile WBC | `control_base_frame`+`marker` must be `world`; vr_follow stays footprint |
| tensors invalidated mid-run | Don’t flip EE/Physics variants on Play; fix snap first |
| `get_ros2_control_robot_description returned empty` (G1D / caster xacro) | Post-guard partial AMENT + stub `/opt/ros/.../component_models` shadows ros2_ws. `_prepare_host_ros_env` must clear AMENT/COLCON/CMAKE/PYTHONPATH before `source setup.bash` so overlay wins |
| `dlopen … libtracetools.so: cannot open shared object file` / ocs2_wbc load fail | `./run.sh` must use `FASIM_FULL_HOST_ROS=1` (keep `/opt/ros` on LD) + exclude pink. Do **not** strip underlay LD. `ensure_isaac_ros2_library_path` keeps ws `lib/` before `/opt/ros` so apt `libocs2_*` cannot shadow |
| apt `libocs2_*.so` shadows workspace (pinocchio SONAME mismatch) | Overlay-first LD; or remove apt ocs2 packages. Workspace sourced ⇒ its `install/*/lib` must precede `/opt/ros/*/lib` |

## Related

- `USDA-OCS2-PhysX-Mujoco` — articulation / Physics variants / DiffDrive
- `fasim-dexhand-asset` — EE Assembler + PhysX hand baseline
- robot_common_launch: `prepare_arms_target_manager_parameters`, `parse_task_info`
