# FaSim URDF → Isaac import — reference

Durable detail from **Zerith H1** (`~/zerith_isaac_import`) after the official Isaac Sim importer. Keep SKILL.md short; put recipes here.

## A. Official commands

No robot importer script. `glb_to_obj.py` is mesh-format only (writes linear `Kd`). Do not use it as a part splitter.

```bash
# expand xacro (URDFImporter does not parse xacro)
# body example: type:=none chassis_joints_movable:=true isaac:=true  (no camera shells)
# gripper example: type:=gripper camera:=d405 isaac:=true; strip left_ prefix if extracting one side

~/isaacsim/python.sh <path-to>/glb_to_obj.py \
  <description>/meshes ~/zerith_isaac_import/meshes
# retarget the prepared URDF mesh filenames to those .obj files, then:
~/isaacsim/python.sh \
  ~/isaacsim/standalone_examples/api/isaacsim.asset.importer.urdf/urdf_import.py \
  --urdf ~/zerith_isaac_import/urdf_src/import/Zerith_H1.urdf \
  --usd-path ~/zerith_isaac_import/final \
  --no-merge-fixed-joints \
  --robot-type "Mobile Manipulators"
```

Then: fold `*_1` siblings into `visuals/` + `collisions/` → rewrite `materials.usda` (§F) → EE / ROS / Sensor on the USDA. Copy into FaSim only after the look matches. Do not wrap this in a Zerith importer.

`base_link_1` (and `head_link2_1`, `link7_1`, …) is USD uniquification when visual and collision share a mesh stem. It is **not** Isaac Sim’s recommended structure and **not** a second chassis. Fold it away; the rigid keeps the original link name.

## B. Mesh scale bake (mirror arms)

URDF often has `scale="1 -1 1"` on right-arm / mirrored finger meshes.

**Bug:** forcing `xformOp:scale=(1,1,1)` drops the mirror → wrong visual + wrong hull.

**Fix** (same idea as `fasim-usd-bake-scale`): multiply points by scale, fix normals, flip winding on odd reflections, author unit scale only. Do this before PhysX cook. The official importer may already bake right-arm mirrors onto mesh instances — leave those; do not strip them. If a collision **wrapper** carries `scale=(1,-1,1)` (Zerith gripper right jaw), copy that xform onto `collisions/` when folding wrappers.

PhysX cooks the prototype mesh and **drops** a negative instance scale, so a part that shares the other side's mesh and only mirrors with `scale=(1,-1,1)` shows a collision facing the other way. Bake a separate mirrored collision mesh (`*_mirrorY`) and set that collision scale to `(1,1,1)`. Leave the visual scale so the render stays mirrored. Zerith: right shoulder pitch/roll, elbow, wrist yaw/roll/pitch, and gripper `jaw_right_finger_link`.

## C. Collision lessons

1. **Do not** turn Zerith’s hollow `base_link` column into one solid hull. That hull reached the shoulder (green pillar beside the left arm). Use `convexDecomposition` with **`maxConvexHulls = 2`** on the main deck collision (`Instances/dipan_link_1`). Do not clip z≤0.33.
2. AABB size-only checks miss offset ghosts; always compare **centers**.
3. Left-arm “body” green on the old split asset was the chassis column, not arm meshes. The single-tree import has no Chassis/Arm prims.
4. Collision **groups** `visuals/` / `collisions/`: `instanceable=false`. Mesh instances may stay instanceable.

## D. Sensor / ROS lessons

| Issue | Fix |
|-------|-----|
| Sensor=none still shows D435 | Camera shells lived in URDF visuals — strip before import; load library USD only in Sensor=sensors |
| Sticky Sensor/ROS on mounts | `prepend variantSets` order: Physics, ROS, Sensor **before** EE |
| Full-tree TF frame names | Remove joint-state PublishTF; odom graph only |
| Late subscriber misses static TF | `staticPublisher=0` on all three TF publishers |
| `cmd_vel` DiffDrive | Prefer **official** Graph (match Tracer V2 / Woosh Ai2): `SubscribeTwist` → `ScaleToFromStageUnit(linear)` → `BreakVector3` (`lin.x`, `ang.z`) → `isaacsim.robot.wheeled_robots.DifferentialController` → `ArticulationController`. Zerith: `wheelRadius=0.0835`, `wheelDistance=0.379`. Math \(\omega_L=(2v-\omega L)/(2r)\), \(\omega_R=(2v+\omega L)/(2r)\); `VelocityCommand=[left,right]`. Identity +Y both drives → **same-sign** (negating right swaps forward↔spin). Wire DiffController + ArticulationController `execIn` from `OnPlaybackTick`, not only from SubscribeTwist |
| DiffDrive saturation | `max*=0` = unset (internal ~1e7). **Never** set a tight `maxWheelSpeed` — Isaac clips L/R **independently**, so both hitting the cap kills Δω (looks like “only turns at low v”). Cap body motion with `maxLinearSpeed` / `maxAngularSpeed` instead. URDF `newton:velocityLimit≈132` (~11 m/s) is usually not the teleop bottleneck |
| Drive PhysX | Keep **`acceleration`**, `K=0`, `D≈1e6` (heavy humanoid; Tracer often `1e5`), `maxForce=inf`, `physxJoint:maxJointVelocity` high (`5e5`). Do **not** copy Woosh `force`+`maxForce=50` onto a ~90 kg wheeled humanoid |
| Casters | **Strip `PhysicsDriveAPI`** — JointState only + free swivel/roll. AgileX Tracer still authors caster Drive with `K=100` (centers forks) — fine for light AGV, fights arc-turn on heavy humanoids. Woosh “casters no Drive” is the right pattern here |
| Friction | Drive μ≈1 `combine=average` (Tracer `wheel_material`). Free casters μ≈0.5 average (not ice once Drive is gone). If commanded ωL≠ωR but chassis will not yaw → contact scrub / tracking, not DiffDrive math |
| Mobile `root_joint` | Official importer welds `/Robot`→`base_link` FixedJoint → **fixed base**. Set `root_joint` `active = false` for wheeled robots |
| Drive vs caster height | All four casters share the same offsets (`caster_z=-0.154`, `wheel_z=-0.008`); drive axle −0.1036 ≈ **0.1 mm** lower (preload). Do not keep URDF rear `wheel_z=-0.007`. Avoid multi‑cm seesaw |
| Articulation solver | Wheeled humanoid / Split Aloha / Woosh: `solverPosition=64`, `solverVelocity=4`, `sleepThreshold=5e-5`, `stabilizationThreshold=1e-5`. Plain chassis-only often stays 32/1 |
| JointState read | Prefer `isaacsim.sensors.physics.IsaacReadJointState` on `base_link` (not deprecated `targetPrim` joint-state path) |
| Nested EE ArticulationRoot | Delete child gripper ArticulationRoot APIs when mounting EE under tcp |
| Prim layout | Flat links under the robot root + `joints`, same as Inex/Luna. Bake each link xform to **world** when lifting it out of `Geometry` (joint `localPos` stays in the body frame). `tcp` / `neck_camera_link` stay under the parent link. Delete empty root `Looks` / `VisualMaterials` |
| targetPrim | `/Zerith_H1/base_link` |

## E. Gripper mimic (Zerith)

- Jaws are prismatic on Y. PhysX: `PhysxMimicJointAPI:transY`, gearing `-1`. Commanded joint rest is **closed**: limits `[0, 0.04]` (`0` closed, `0.04` open). Mimic finger limits `[-0.04, 0]`. Finger link translates are shifted to that closed rest (`left` Y `-0.04`, `right` Y `+0.04`) so drive target `0` holds closed.
- MuJoCo: `newton:mimicCoef1=-1` so `q_finger = -q_gripper`.
- Command only `*_gripper_joint`.
- Side deactivates importer `joints/gripper_joint` and `jaw_right_finger_joint`, then authors `left_*` / `right_*` copies. Gripper links are root siblings (`gripper_base`, fingers, `jaw_camera_link`, `eef`), same flatten as the robot.
- Mimic used `rotY` on the old split asset — wrong; these joints are transY.

## F. Materials after official import

`emission = emissive_factor * max(0, emissive_strength)` in FaSim `Textures/pbr.mdl` `gltf_material`.

The shipped importer (converter 0.3.2) is the wrong look if left as-is:

1. It does not read `.glb`. `glb_to_obj.py` writes linear `baseColorFactor` into MTL `Kd`.
2. The importer treats `Kd` as sRGB and stores linear `UsdPreviewSurface.inputs:diffuseColor`. Dark colors become ~12.92× too dark (`0.01176` → `0.00091`).
3. It drops `emissiveFactor`, metallic (Zerith grey `0.712`), and distinct roughness (motor `0.25` vs rubber `0.5`).
4. Same base color collapses to one prim. Zerith `black` (face) and `black metal` (wrists, `link6`) both landed on `Materials/black_2`. Gripper jaw `black rubber` was referenced as `Materials/motor`.

Fix on the USDA, not with another importer. Copy `Textures/pbr.mdl`. Rewrite `payloads/materials.usda` to `gltf_material` using the **previous asset’s** `base_color_factor`, not the PreviewSurface color. Instance xforms already `references = @./materials.usda@</Materials/NAME>`; keep those names. Split only the collapsed bindings.

Zerith body (`robots/humanoid/Zerith/H1/payloads/materials.usda`):

| Prim | Was | base_color_factor | metal | rough | Extra |
|------|-----|-------------------|-------|-------|--------|
| `dark` | body / head shell | `(0.011764706, 0.011764706, 0.015686275)` | 0 | 0.5 | |
| `purple` | LED | `(0.2509804, 0.24705882, 0.8235294)` | 0 | 0.5 | `emissive_factor (0.25015837, 0.24620144, 0.8227861)`, `emissive_strength 5000` |
| `silver` | arm / trim | `(0.7764706, 0.7882353, 0.8)` | 0 | 0.5 | |
| `grey` | `body_link3` | `(0.05490196, 0.0627451, 0.09019608)` | 0.7123288 | 0.5 | |
| `black` | rubber: wheels, `link1` | `(0.011764706, 0.011764706, 0.011764706)` | 0 | 0.5 | |
| `black_2` | black metal: wrists, `link6` | `(0, 0, 0)` | 0 | 0.5 | |
| `black_face` | face only | `(0.016, 0.018, 0.026)` | 0.12 | 0.05 | IOR 1.52, clearcoat 1, clearcoat roughness 0.04, specular `(0.92, 0.95, 1)` |

Point only `neck_pitch_link_1_1` (`head_link2.glb` material `black`) at `Materials/black_face`. Leave wrist / `link6` on `black_2`.

If only the left head LED glows, the right-side faces of `neck_pitch_link_2` are wound inward — flip those faces (Zerith: 38 faces with mean Y<0). The material was already bound.

Gripper: `motor` roughness 0.25 (same base as rubber); `black` is flange plastic `(0.003921569, 0.003921569, 0.003921569)`; `black_1` is black metal; retarget `right_jaw_left_finger_link` from `Materials/motor` to `Materials/black_rubber` (roughness 0.5).

GUI: **Reload Stage**. A later `urdf_import.py` overwrites `materials.usda`.

## G. EE mounts (Zerith)

Child under `Geometry/.../left_tcp` or `right_tcp` (`translate` 0.046 on the tcp). Right-arm negative scale is on mesh instances, not on the tcp parent.

| EE | quat (w,x,y,z) | translate | child |
|----|----------------|-----------|--------|
| gripper | identity | 0 | `gripper_base`; Sensor=d405; Side=left\|right; Robot=none; FixedJoint to `gripper_base` |
| left revo1 | `(0.5, 0.5, 0.5, -0.5)` | 0 | `hand_base`; deactivate `root_joint` |
| left revo2 | `(0.5, -0.5, 0.5, -0.5)` | `(0.02, 0, 0)` | wrist→`hand_base` 0.066 = tcp 0.046 + 0.02 |
| right revo1 | `(0.5, -0.5, -0.5, -0.5)` | 0 | |
| right revo2 | `(0.5, 0.5, 0.5, 0.5)` | `(0.02, 0, 0)` | |

## H. Example PhysX gain hooks (Zerith)

Author on the PhysX USDA. Revolute values below are per-degree (importer × π/180 if set through the Python API). Do not pass wheel damping into MuJoCo.

| Joint class | K | D | Notes |
|-------------|---|---|--------|
| Passive caster/wheel | — | — | **No DriveAPI**; JointState only; free swivel/roll |
| Drive wheels | 0 | 1e6 | PhysX **acceleration**; `maxForce=inf`; high `maxJointVelocity` |
| Gripper prismatic | 10000 | 1000 | N/m |
| Body lift prismatic | 60000 | 6000 | N/m |
| Arm / head revolute | 60000 | 6000 | per-degree USDA |

MuJoCo PD separate; do not copy PhysX wheel damping into actuators.

## I. Open pitfalls

1. Do not revive plate+column clip scripts or part-split Chassis/Body/Head/Arm for Zerith. Decomp with two hulls on `base_link` collisions is the agreed chassis approximation.
2. DexForce W1 / Airbot MMK2 / X2Robot / airbot_play — import not finished in this playbook; negative-scale mirrors still need bake.
3. Zerith ROS topics are `/isaac/joint_*`; align per robot / WBC before claiming “enable done”.
4. Do not treat `/tmp` backups as truth — `diff -rq` staging vs FaSim before editing.
5. DiffDrive: do not reintroduce ScriptNode kinematics or negate the right wheel “to fix” signs without checking joint `localRot` first.
6. Do not copy AgileX Tracer caster `Drive+K=100` or Woosh drive `force+maxForce=50` onto wheeled humanoids without re-validating arc-turn at mid/high `v`.
