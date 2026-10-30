---
name: fasim-urdf-isaac-import
description: >-
  Practical playbook for importing humanoid/mobile-manipulator xacro into
  FaSim-Isaac Isaac Sim 6.2 USD via the shipped URDFImporter + Asset
  Transformer. Do not part-split (no Chassis/Body/Head/Arm trees, no
  build_final). After import, fold *_1 collision siblings into visuals/
  collisions, flatten links to root siblings + `joints` (Inex/Luna),
  replace UsdPreviewSurface with Textures/pbr.mdl (the importer
  double-gamma-corrects OBJ Kd and drops emission), then add EE, ROS, Sensor
  on the USDA. Wheeled: official DifferentialController Graph, caster no-Drive,
  maxWheelSpeed traps. Points to isaac-urdf-usda-ocs2 and
  fasim-robot-mujoco-physics for Assembler/OCS2/MuJoCo depth; do not duplicate those.
---

# FaSim URDF → Isaac Sim Import

Official path only: offline xacro → OBJ if needed → Isaac Sim `urdf_import.py` → fold collision siblings into `visuals/` + `collisions/` → USDA supplements (materials, EE, ROS, Sensor). Do not write a project importer, and do not split a whole robot into Chassis / Body / Head / Arm.

**Companion skills (do not re-copy their content):**
- `USDA-OCS2-PhysX-Mujoco` (`isaac-urdf-usda-ocs2`) — Assembler, Physics/Side variants, OCS2, 4WS
- `fasim-robot-mujoco-physics` — MuJoCo / Newton strip Drive, Side actuators, gains
- `fasim-usd-bake-scale` — bake non-unit `xformOp:scale` into mesh points
- `fasim-rg75-pad-convert` / `fasim-dexhand-asset` — gripper / dexhand specifics

**Reference robots**

| Robot | Path |
|-------|------|
| Zerith H1 | `robots/humanoid/Zerith/H1/` — one-tree official import |
| Inex | `robots/humanoid/Rokae/Inex/` — `visuals/` + `collisions/` layout |
| Ai2 Bot2 | `robots/humanoid/Ai2/Bot2/` — Sensor / ROS patterns |

**Tooling**

| Role | Location |
|------|----------|
| Isaac Sim 6.2 | `~/isaacsim` (`~/isaacsim/python.sh`) |
| Official URDF import | `~/isaacsim/standalone_examples/api/isaacsim.asset.importer.urdf/urdf_import.py` |
| Official skill | `~/isaacsim/skills/urdf-mjcf-to-usd-conversion/SKILL.md` |
| Mesh format only | `glb_to_obj.py` converts `.glb` → `.obj` (converter does not read GLB). Not a robot builder. |
| Descriptions | `~/ros2_ws/src/robot-descriptions/` — **xacro is the only source** |

**Do not**
- `git commit` / `git push` unless the user explicitly asks
- Touch `robots/humanoid/FiveAges/Gen2` or `Gen3`
- Mutate URDF提取’s dirty worktree; prefer `git show` into a temp dir
- Recreate part-split builders (`build_final.py`, `build_zerith.py`, Chassis/Body/Head/Arm trees)

---

## When to use

- New FaSim humanoid from `robot-descriptions` xacro (Zerith, X2Robot, DexForce, Airbot, …)
- Rebuild after chassis DOF / mesh / sensor frame changes
- Fix: importer `*_1` collision siblings, Sensor=none still showing cameras, full-tree TF, mirror arm wrong, purple not emissive, hollow-column solid hull

Use **isaac-urdf-usda-ocs2** instead when assembling **shared category** arms/wheels via Robot Assembler + adapter payloads (Galaxea, Cobot Magic, Split Aloha).

---

## Checklist

```
- [ ] 1. Expand xacro only; chassis_joints_movable / type / camera flags as agreed
- [ ] 2. Strip camera meshes from body URDF; Sensor=sensors loads library realsense USD
- [ ] 3. Convert .glb → .obj (`glb_to_obj.py`); point URDF at OBJ (converter ignores GLB)
- [ ] 4. Official `urdf_import.py` (`--no-merge-fixed-joints`, robot_type as needed). One tree, then flatten to root-level links + `joints` (Inex / Luna). ArticulationRoot on `base_link`
- [ ] 5. Fold `*_1` collision siblings into per-link `visuals/` + `collisions/` (not a second rigid)
- [ ] 6. Root prepend variantSets: Physics, ROS, Sensor, EE_*; Robot last/append
- [ ] 7. Bake URDF scale="1 -1 1" into points (never leave xformOp:scale=-1 on a rigid/tcp parent)
- [ ] 8. Default convexHull. Zerith `base_link/collisions` main mesh: convexDecomposition, maxConvexHulls=2
- [ ] 9. Replace UsdPreviewSurface with Textures/pbr.mdl; restore emission / metal / split collapsed blacks
- [ ] 10. Gripper: drive *_gripper_joint; mimic finger; PhysX PhysxMimicJointAPI + MuJoCo mimic
- [ ] 11. ROS enable = joint + official DiffDrive `cmd_vel` if wheeled + odom-only TF. Deactivate importer `Physics/root_joint`. Casters strip DriveAPI; drive wheels `acceleration` + high D; `maxWheelSpeed=0`
- [ ] 12. Validate: no Chassis/Arm prims; one ArticulationRoot; AABB center+size; Sensor none/sensors; EE=none leaves tcp empty
```

---

## 1. Source of truth

1. **Only expand xacro** (`robot.xacro` / package entry). Flat `urdf/` dumps may be stale.
2. Coordinate flags with URDF提取: e.g. `type:=none`, `chassis_joints_movable:=true`, `isaac:=true`, no camera shells on the body.
3. Gripper is a **separate** URDF / USD (`type:=gripper`), mounted later as EE, not a body part.

---

## 2. Official import

Isaac Sim 6.2 converter (`urdf_usd_converter` 0.3.2) reads STL/OBJ/DAE only. `.glb` is dropped with empty meshes.

```bash
# 1) expand xacro offline (URDFImporter does not parse xacro)
# 2) convert meshes if the description ships GLB
~/isaacsim/python.sh <path-to>/glb_to_obj.py \
  <description>/meshes <out>/meshes
# 3) point the prepared URDF mesh filenames at those .obj files, then:
~/isaacsim/python.sh \
  ~/isaacsim/standalone_examples/api/isaacsim.asset.importer.urdf/urdf_import.py \
  --urdf <expanded.urdf> \
  --usd-path <out> \
  --no-merge-fixed-joints \
  --robot-type "Mobile Manipulators"
```

Keep `merge_fixed_joints=False` so `tcp` / `eef` / camera frames survive. Do not wrap this in a robot-specific importer.

Importer tree is `Geometry/<link>` nested, joints under `Physics`. FaSim robots (Inex, Luna, grippers) flatten that before shipping: every URDF link is a **root sibling** with its **world** xform, `tcp` / camera frames stay under the parent link, joints live in `joints`. Drop empty root `Looks` and `VisualMaterials`; real bindings stay on mesh instances. Do not leave Zerith on the importer `Geometry` wrapper. Do not part-split into Chassis/Arm.

---

## 3. `visuals/` + `collisions/` (required)

CLI `urdf_import.py` writes same-stem visual/collision as **sibling** Xforms (`base_link` visual, `base_link_1` collision, `purpose=guide`). That `_1` is USD name uniquification (`name_cache.getPrimNames`), **not** a second rigid and **not** the Isaac Sim recommended sim structure.

Isaac Sim 6.2 [Asset Structure](https://docs.isaacsim.omniverse.nvidia.com/6.2.0/robot_setup/asset_structure.html) nests links under `Geometry/`. FaSim ships the flattened Inex / Luna form instead:

```text
/<Robot>/
  <link>/                 # root sibling, world xform
    visuals/              # render meshes, purpose default, instanceable=false
    collisions/           # collision meshes, purpose=guide, instanceable=false
    <tcp, eef, camera frames>   # stay nested, local xform
  joints/                 # renamed from importer Physics
```

Bake each lifted link to its **world** pose (`W = local * parentWorld`). Joint `localPos0/1` stay in the body frame — do not recompute them. After import, **before** copying into FaSim:

- Move visual wrapper children into `visuals/`; move `*_1` collision-wrapper children into `collisions/`; delete the wrappers.
- Do **not** put `tcp`, `eef`, `head_camera`, or nested links into `visuals/`.
- Primitive wheel Sphere/Cylinder with `purpose=guide` go under `collisions/`.
- Mesh instances may stay `instanceable`; the `visuals/` and `collisions/` groups must not.
- Copy wrapper xforms (including a finger `scale=(1,-1,1)`) onto the destination group so the mirror is not dropped.

Align Inex / CR5: same group names, same purpose.

---

## 4. Collisions

| Rule | Detail |
|------|--------|
| Default | `convexHull` per link |
| Zerith `base_link` | Main deck collision (`Instances/dipan_link_1`): `convexDecomposition`, `maxConvexHulls = 2` (plate + column). Do **not** clip z≤0.33 and do **not** one solid hull for the hollow column |
| Concave else | bounded hull count; few large pieces |
| Mirror | URDF `scale="1 -1 1"` → bake into points/normals/winding; **never** leave `xformOp:scale=-1` on a rigid or tcp parent |
| Validate | World AABB **size and center** vs visual |

---

## 5. Variants

```usda
prepend variantSets = ["Physics", "ROS", "Sensor", "EE_Left", "EE_Right"]
append variantSets = ["Robot"]
```

Physics / ROS / Sensor first so mount payloads cannot sticky-override them. Do not sticky Physics on EE mounts (VariantSwitcher). Nested child Physics is pushed from the parent Physics payload when needed.

| Set | Options (typical) | Notes |
|-----|-------------------|--------|
| Physics | `none` / `physics` / `physx` / `mujoco` | PhysX first for OCS2 |
| ROS | `none` / `enable` | joint + odom TF (+ `cmd_vel` if wheeled) |
| Sensor | `none` / `sensors` | none = **no** camera prims; wrist D405 follows the gripper |
| EE_* | `none` / `gripper` / `revo1` / `revo2` | child under `tcp`; deactivate child `root_joint` |
| Robot | `none` / `robot` | IsaacRobotAPI; mounts use `Robot=none` |

---

## 6. Materials after official import

Do **not** keep the importer’s `UsdPreviewSurface`. Converter treats MTL `Kd` as sRGB while `glb_to_obj.py` already stored linear `baseColorFactor`, so dark colors come out ~12.92× too dark. Emission, metallic, and per-material roughness are dropped; same base color collapses to one prim.

After import: copy `Textures/pbr.mdl`, rewrite `payloads/materials.usda` to `gltf_material` using the **previous asset’s** `base_color_factor`. Keep instance references to `Materials/NAME`. Split only collapsed bindings (Zerith face glass vs wrist black metal). Purple LED needs **both** `emissive_factor` and `emissive_strength`. GUI: **Reload Stage**. Re-running `urdf_import.py` overwrites `materials.usda`.

Numbers: [reference.md](reference.md) §F. If one side of a two-sided LED is dark, the winding faces inward — flip those faces, do not rebind the material.

---

## 7. Sensors, EE, ROS

1. `Sensor=none` → zero Camera prims. `Sensor=sensors` → library USD on the empty camera frame (Zerith head: `neck_camera_link/head_camera` + whole `robots/sensors/realsense/d435.usd`, **not** `</World/d435>` or Looks/materials vanish). Mount Rx90+Rz90 + `rotateX:unitsResolve` like Open_Nero. Optical Camera is a **sibling** of the mesh, quat `(0, 1, 0, 0)` look −Z.
2. Wrist D405 lives on the **gripper** `Sensor=d405` at `jaw_camera_link`, not on the body. Payload whole `robots/sensors/realsense/d405.usd` (Y-up: `rotateX:unitsResolve=90` + **Ry=-90** so the lens faces the link); optical Camera is a **sibling**, look −Z.
3. EE child under `tcp`. Gripper: identity, child `gripper_base` (importer does not rename it to `base_link`), `Sensor=d405`, `Side=left|right`, `Robot=none`, FixedJoint to `gripper_base`. Revo quats/translates: [reference.md](reference.md).
4. Gripper Side renames importer `Physics/gripper_joint` → `left_gripper_joint` / `right_gripper_joint`. Mimic is prismatic Y: `PhysxMimicJointAPI:transY`, gearing `-1`.
5. ROS `targetPrim` / `chassisPrim` = `/Robot/base_link` (flat link, not `/Robot/Geometry/base_link`). Publish `/isaac/joint_states`, `/isaac/joint_command`, clock. TF **only** `world → odom → base_footprint → base_link`, all three `staticPublisher=0`. Semantics: `odom→*` = `IsaacComputeOdometry` **relative** from Play; `world→odom` = **Play latch** of chassis world pose minus footprint→base z (so `lookup(world, base_link)` matches stage at start — do **not** publish identity `world→odom`, and do **not** write every-frame current base into `world→odom` or it double-counts). Wheeled: **official** `ROS_Differential_Controller` (Tracer V2 / Woosh Ai2 topology) — not a ScriptNode. Measure `wheelRadius` / `wheelDistance`; `jointNames` = `[left, right]`; identity +Y both drives → same-sign cmds (do not negate right). Casters: **no** `PhysicsDriveAPI`. See [reference.md](reference.md) §D. G1D fiveages example: `ComputeWorldChassis` → ArrayIndex → `LatchWorld2Odom` ScriptNode → `TFWorld2Odom`.

---

## 8. Validation

| Check | Pass |
|-------|------|
| Composition | No Chassis/Body/Head/Arm_Left prims; one `ArticulationRoot` |
| Structure | Each rigid has `visuals/` + `collisions/`; no `base_link_1` sibling |
| EE none | `tcp` empty except sites (`*_eef`) |
| Sensor none | 0 Camera; sensors: expected library cameras |
| TF | Only odom chain; `staticPublisher=0`; `world→odom` latched spawn (minus footprint z), `odom→*` relative |
| Emissive | LED factor × strength > 0; both sides face outward |

---

## Additional resources

- [reference.md](reference.md) — materials table, EE quats, ROS/mimic pitfalls
- Skills: `USDA-OCS2-PhysX-Mujoco`, `fasim-robot-mujoco-physics`, `fasim-usd-bake-scale`
- NVIDIA: [Asset Structure](https://docs.isaacsim.omniverse.nvidia.com/6.2.0/robot_setup/asset_structure.html)
