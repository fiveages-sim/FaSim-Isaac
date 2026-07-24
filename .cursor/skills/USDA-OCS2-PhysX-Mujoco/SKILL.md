---
name: isaac-urdf-usda-ocs2
description: >-
  Converts URDF/xacro to Isaac Sim USDA, assembles multi-part robots with Robot
  Assembler, builds Physics/Side/component variants, and tunes PhysX + MuJoCo
  layers for stable OCS2 / ros2_control. Use when importing URDF, Asset
  Transformer, Robot Assembler, variantSets, VariantSwitcher, Physics=physx|
  mujoco, gripper mimic/drive, AdaptiveGripperController, Newton vs PhysX, or
  ARX Lift2S / LunarBot style mobile manipulators.
---

# Isaac URDF → USDA → Variants → OCS2 Physics

End-to-end workflow for ROS robots in **Isaac Sim 6**: import, assemble, variantize, and stabilize **PhysX** + **Newton/MuJoCo** for **OCS2** / topic-based `ros2_control`.

**Companion skills:** For deep MuJoCo-only porting patterns see project skill `fasim-robot-mujoco-physics` if present. For gripper/PhysX close pitfalls see [reference.md](reference.md).

**Reference assets**

| Robot | Path |
|-------|------|
| ARX Lift2S 6.0 | `FaSim-Isaac/robots/mobile_manipulator/ARX_Lift_2S_6.0/ARX_LIFT2S/` |
| Old ARX Lift2S PhysX | `FaSim-Isaac/robots/mobile_manipulator/ARX_Lift2S/` + `manipulators/ARX/ARX5_Gripper_2025/configuration/left_physic.usd` |
| Notes | `Isaac-Intern-Folder/LunarBot/notes/URDF-to-USD-USDA.md` |

## Checklist

```
- [ ] 1. Split URDF parts; expand xacro; record mount poses
- [ ] 2. Isaac 5: URDF → USD; Isaac 6: Asset Transformer → USDA
- [ ] 3. Robot Assembler: child→parent; merge articulation (delete child roots)
- [ ] 4. Root variantSets + local adapter payloads (not direct category refs)
- [ ] 5. Physics=none|physics|physx|mujoco; Physics first in variantSets
- [ ] 6. Nested Physics propagation OR VariantSwitcher-friendly (no sticky child Physics on mounts)
- [ ] 7. Env selections match variant *option* names after renames
- [ ] 8. PhysX drives/mimic + Side robotJoints; MuJoCo strip Drive + MjcActuator
- [ ] 9. Validate OCS2 open/close + arms under PhysX and Newton separately
```

---

## 1. URDF → USD → USDA

### 1.1 Prepare

- Import **one articulation per file** (chassis, base, arm, gripper, wheel).
- Expand xacro to flat URDF; fix mesh paths.
- Record fixed-joint mount **translate + RPY** from the full robot xacro before assembly.

### 1.2 Convert

| Step | Isaac | Action |
|------|-------|--------|
| Import | **5.x** | File → Import URDF → per-component output folder |
| Transform | **6.0.x** | Open USD → Tools → Robotics → Asset Transformer → Isaac Sim Structure → Execute |
| Cleanup | 6 | Keep `.usda` + `payloads/` + `Textures/`; drop old `.usd` / `configuration/` when USDA loads |

Prefer file import over ROS2 URDF node (Fast DDS vs Zenoh mismatches).

### 1.3 Physics after transform

| Joint role | Drive | Stiffness | Damping |
|------------|-------|-----------|---------|
| Position (arms, lift, steer) | force + targetPosition | tuned | tuned |
| Continuous wheel | force + targetVelocity | **0** | high (e.g. 1e5–1e6) |
| Mecanum | same as wheel + `isaacmecanumwheel:radius/angle` | | |

---

## 2. Robot Assembler

**Tools → Robotics → Asset Editor → Robot Assembler**

1. Attach **child → parent** with xacro mount pose (prefer named robot links/sites).
2. Set **Assembly Namespace** to the prim name used in variants (e.g. `AC_One_Base`, `Arm_Left`).
3. On the child: **delete** `PhysicsArticulationRootAPI` / `PhysxArticulationAPI` / `NewtonArticulationRootAPI` from child root; Assembler adds `AssemblerFixedJoint`.
4. Parent should keep a single articulation root (e.g. chassis `base_link`).
5. Merge DOFs with `isaac:physics:robotJoints` / `robotLinks` pointing at composed child paths.
6. **Do not** hot-swap gripper EE mid-sim (PhysxMimicJoint fails). Assemble with EE selected, or stop sim before changing EE.

```usda
over "base_link" (
    delete apiSchemas = ["PhysicsArticulationRootAPI", "PhysxArticulationAPI", "NewtonArticulationRootAPI"]
) {
    def PhysicsFixedJoint "AssemblerFixedJoint" {
        rel physics:body0 = </Parent/.../mount_link>
        rel physics:body1 = </Parent/.../Child/base_link>
        /* localPos / localRot from xacro */
    }
}
```

---

## 3. Variants

### 3.1 Two-layer pattern

```text
Robot.usda          ← variantSets select LOCAL adapters
  payloads/
    Arm_Left/X5_Arm.usda   ← mount + FixedJoint + payload → shared X5.usda
    Wheel/...              ← adapter → components/omnia_150/
    Physics/{none,physics,physx,mujoco}.usda
    ROS/, Sensor/, Robot/
```

- Root variants point at **`payloads/…` adapters**, not directly at shared category assets.
- Adapter files own mount xforms and assembler joints; category assets stay reusable.

### 3.2 Register sets

Typical sets: `Physics`, `Wheel`/`Wheel2`/`Wheel3`, `AC_One_Base`, `Arm_Left`, `Arm_Right`, `ROS`, `Sensor`, `Robot`, plus nested `Side` / `EE` on arms/grippers.

```usda
prepend variantSets = ["Physics", "Wheel", "AC_One_Base", "Arm_Left", "Arm_Right", "ROS"]
```

**Physics before Side** on grippers/arms so `Physics=mujoco` can strip Side’s `PhysicsDriveAPI`.

### 3.3 Rename variant options safely

If you rename option `"base"` → `"omnia_150"` / `"AC_One_Base"`:

1. Update **every** env that selects that option (`env/empty.usda`, etc.).
2. Keep **prim path** names (`over "omnia_150"`, `over "AC_One_Base"`) consistent with payload `defaultPrim`.
3. Re-check nested Physics overs still target those prim names.

Mismatch → payloads never load → Physics propagation and OCS2 control look “broken”.

### 3.4 VariantSwitcher (Newton ↔ PhysX)

Isaac `omni.physics.isaacsimready` **VariantSwitcher** sets `Physics` on the **session layer**:

| Simulator | Physics variant |
|-----------|-----------------|
| PhysX | `physx` |
| Newton | `mujoco` |

Rules:

- Do **not** sticky `Physics=` on the edit/authoring layer of env files for prims you want auto-switched.
- Do **not** bake conflicting child `Physics=` on arm/EE **mount** adapters (fights session clears).
- Composite roots may **push** child Physics inside the root’s Physics variant body so nested arms/grippers/wheels flip with the parent:

```usda
variantSet "Physics" = {
    "physx" ( prepend payload = @./payloads/Physics/physx.usda@ ) {
        over "lift_link" {
            over "AC_One_Base" ( variants = { string Physics = "physx" } ) {
                /* … Arm_Left / Gripper_2025 Physics = physx … */
            }
        }
        over "wheel_1" {
            over "omnia_150" ( variants = { string Physics = "physx" } ) {}
        }
    }
    "mujoco" ( prepend payload = @./payloads/Physics/mujoco.usda@ ) { /* same with mujoco */ }
}
```

---

## 4. Physics layers

| File | Role |
|------|------|
| `physics.usda` | Shared mass, RB, joints, limits |
| `physx.usda` | `subLayers = [@./physics.usda@]` + DriveAPI, PhysxMimic, solver |
| `mujoco.usda` | `subLayers = [@./physics.usda@]` + strip Drive + MjcJoint/MjcActuator / Newton mimic |
| `none.usda` | No physics |

**Never** mix PhysX-only and MuJoCo-only params in one unchecked layer. **Never** edit `mujoco.usda` when fixing PhysX-only OCS2 bugs (and vice versa).

### 4.1 PhysX (OCS2 / ArticulationController)

- Position joints: `drive:*:physics:type = "force"`, stiffness/damping/maxForce tuned.
- Single articulation root on chassis; children roots deleted.
- Grippers: see [reference.md](reference.md) § Gripper PhysX + OCS2.
- Side left/right: `robotJoints` must list **active** renamed joints (`left_gripper_joint`), not deactivated `gripper_joint`.

### 4.2 MuJoCo / Newton

- `delete` Drive/JointState APIs; block residual `drive:*` attrs (`= None`).
- Control via `MjcActuator` + `MjcJointAPI`; use `mjc:actuatorgravcomp` where needed.
- Mimic: `NewtonMimicAPI` / MJC mimic with URDF **multiplier** (often `+1`), independent of PhysX `gearing`.

---

## 5. OCS2 / ros2_control bridge

Typical stack:

- Isaac OmniGraph: `ROS2SubscribeJointState` → `IsaacArticulationController` on chassis `base_link`.
- Publish `joint_states` via `IsaacReadJointState`.
- Host: `topic_based_ros2_control` + `ocs2_arm_controller` + `adaptive_gripper_controller`.

Gripper AdaptiveGripperController:

| Command | Meaning | Force feedback |
|---------|---------|----------------|
| `0` | close | **ON** (`force_threshold`, often `0.5`) |
| `1` | open | OFF |

PhysX `maxForce` / projected joint effort must not exceed `force_threshold` during **free-space close**, or close aborts immediately while open still works. Details: [reference.md](reference.md).

---

## 6. Validation

Compose with `isaacsim/python.sh`: set `Physics=physx` and `mujoco`; print child Physics selections and gripper drive/mimic/`maxForce` property stacks (Physics layer must win over Side).

Play checklist:

- [ ] PhysX: arms track OCS2; grippers open **and** close; both fingers move (mimic)
- [ ] Newton: same with `Physics=mujoco`; no PhysX Drive left on actuated joints
- [ ] VariantSwitcher: Newton↔PhysX flips root + nested components
- [ ] Env variant selections match renamed options

---

## Additional resources

- [reference.md](reference.md) — gripper PhysX/OCS2, mimic gearing, VariantSwitcher skip rules, troubleshooting
- Project notes: `Isaac-Intern-Folder/LunarBot/notes/URDF-to-USD-USDA.md`
- NVIDIA: [Asset Structure](https://docs.isaacsim.omniverse.nvidia.com/latest/robot_setup/asset_structure.html)
