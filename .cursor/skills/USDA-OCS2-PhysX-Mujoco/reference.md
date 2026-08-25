# Reference: PhysX / MuJoCo / OCS2 (Isaac USDA)

Deep notes for gripper mimic, AdaptiveGripperController, VariantSwitcher, and common failures. Read from [SKILL.md](SKILL.md) when debugging PhysX vs Newton control.

## Physics variant map

| Simulator UI | USD `Physics` selection | Payload |
|--------------|-------------------------|---------|
| PhysX Simulation | `physx` | `payloads/Physics/physx.usda` |
| Newton Simulation | `mujoco` | `payloads/Physics/mujoco.usda` |

Newton does **not** auto-select `mujoco`; **VariantSwitcher** (or nested overs / manual selection) must flip the asset.

Isolate engine-specific params:

- Shared dynamics → `physics.usda`
- PhysX Drive / PhysxMimic / PhysxArticulation → `physx.usda` only
- MjcActuator / MjcJoint / NewtonMimic / Drive strip → `mujoco.usda` only

When the user says “fix PhysX / do not touch mujoco”, edit **only** `physx.usda` (and Side/`robotJoints` if needed).

---

## Gripper PhysX + OCS2

### Authoritative PhysX reference (ARX)

`FaSim-Isaac/robots/manipulators/ARX/Gripper_2025/payloads/Physics/physx.usda`（旧 crate `ARX5_Gripper_2025/.../left_physic.usd` 已移除；下列数值仍为 PhysX/OCS2 参考基线）:

| Property | Value |
|----------|-------|
| Drive type | `force` |
| stiffness / damping | `60000` / `6000` |
| maxForce (stock) | `20` |
| Mimic | `PhysxMimicJointAPI:rotY` |
| `gearing` | **`-1`** |
| `naturalFrequency` / `dampingRatio` | `0` / `0` (hard mimic) |
| Leader limits | `[0, 0.044]` |
| Follower (joint8) limits (stock) | `[-0.0088, 0.0528]` |

`joint8` often has `localRot = (0,1,0,0)` (180° flip). PhysX `gearing` and URDF `mimic multiplier` are **not** required to match:

| Backend | Typical sign | Why |
|---------|--------------|-----|
| PhysX | `gearing=-1` | Matches flipped joint frame in `left_physic.usd` |
| URDF / Newton | `multiplier=+1` / Newton coef `+1` | MJC/USD path |

Do **not** blindly set PhysX `gearing=+1` to “match URDF” — that can freeze link8 or break stroke unless limits/NF are retuned carefully.

### Side left/right

- Deactivates default `gripper_joint` / `joint8`.
- Defines `left_*` / `right_*` joints + PhysX mimic + (for Newton) MjcActuators.
- **`isaac:physics:robotJoints` must list the active renamed joints**, not the deactivated defaults. Otherwise PhysX articulation merge / Robot Assembler may omit drive DOFs while Newton still works via actuators.

```usda
delete rel isaac:physics:robotJoints
prepend rel isaac:physics:robotJoints = [
    </Gripper/root_joint>,
    </Gripper/joints/left_gripper_joint>,
    </Gripper/joints/left_joint8>,
]
```

Put list-ops **inside** the prim body `{ }`, not in the `def "…" ( … )` specifier (parse error).

Physics layer opinions should **win** over Side for drive/mimic when `Physics=physx` (verify with `GetPropertyStack`).

### AdaptiveGripperController (OCS2)

Source: `ros2_ws/.../adaptive_gripper_controller.cpp`

| `target_command` | Action | Force feedback |
|------------------|--------|----------------|
| `1` | open → upper/lower from URDF initial | **disabled** |
| `0` | close | **enabled** |

On close, if `|effort| > force_threshold` (yaml often `0.5`, `force_feedback_ratio` often `0.1`):

- New target ≈ `current + 0.1 * (closed - current)`
- Looks like “cannot close” / stuck nearly open
- **Open still works** (no effort check)

PhysX `IsaacReadJointState` efforts are projected joint forces; with stiff PD they often sit near **`maxForce`** whenever tracking error is large.

**PhysX-only fix for OCS2 free-space close** (keep mimic `gearing=-1` + stiff kp/kd from `left_physic.usd`):

1. Set drive `maxForce` **below** `force_threshold` (e.g. `0.4` when threshold is `0.5`).
2. Optionally widen follower limits to `[-0.044, 0.044]` so `gearing=-1` at open (`q_leader=0.044` → `q_follower=-0.044`) is not pinned on `-0.0088` (limit fight → effort spike).

Newton often reports near-zero free-motion effort for MJC position actuators, so the same controller works without lowering MuJoCo force ranges — **do not** change `mujoco.usda` for this class of PhysX-only close bug.

### Mimic API mid-sim

Error: `failed to find internal joint object for PhysxMimicJointAPI`

- Do not change EE / load gripper while Play is running after Assembler has merged articulations.
- Finish assembly with EE selected (or stop → set EE → Play).

---

## VariantSwitcher skip / sticky Physics

From `omni.physics.isaacsimready` VariantSwitcher:

- Skips a prim if the **edit target layer** already has a `Physics` variantSelection.
- Session-layer opinions it writes are tracked and can be cleared on switch.
- Parent sticky `variants = { Physics = … }` on children (in mount adapters or env) **fights** session clears → “child won’t switch”.

Prefer:

- No authored `Physics=` on env for auto-switched prims.
- Nested push of child Physics **inside** root `variantSet "Physics"` bodies (follows parent selection), **or** leave children without sticky Physics and rely on VariantSwitcher scanning all `Physics` variantSets.

After renaming variant **options**, update env selections or components never load and nested overs become no-ops.

---

## Articulation merge checklist

| Item | PhysX / OCS2 need |
|------|-------------------|
| One root | Parent `PhysicsArticulationRootAPI` (chassis `base_link` or composite root) |
| Chassis child root | Delete ArticulationRoot on the prim that **has** it (SteerChassis `base_link`; Tracer child root). Galaxea: `PhysicsArticulationRootAPI` only |
| Arm / EE child root | `root_joint` `active = false` **only** — do not delete ArticulationRoot APIs |
| `robotJoints` | Includes chassis + arms + **active** gripper joints under merged tree |
| Controller target | Nested chassis `ArticulationController.targetPrim` → **parent root** (merged DOFs) |
| Odometry | Nested `ComputeOdometry.chassisPrim` → parent `base_footprint` / `base_link` |
| Joint names | Authored `token[]`, matching URDF / OCS2 / wheel names |

---

## Nested chassis ROS (Cobot Magic / Galaxea_R1)

Child chassis graphs (`cmd_vel`, `/odom`) stay on the child asset. After merge they must be **retargeted** from the parent (mount adapter or root `over "ChassisPrim"` like `humanoid/Galaxea/R1/R1.usda`):

```usda
over "ArticulationController"
{
    custom token[] inputs:jointNames = ["left_wheel", "right_wheel"]
    custom rel inputs:targetPrim = </ParentRoot>
}
over "ComputeOdometry"
{
    custom rel inputs:chassisPrim = </ParentRoot/base_footprint>
}
```

Sticky child `ROS=enable` on the chassis mount (or parent root over). Do **not** sticky `Physics` there.

### `Invalid DOF name ()`

`OgnIsaacArticulationController` `get_dof_indices` with an empty string. `targetPrim` may already be the merged parent (Available DOFs include wheels **and** arm joints) — the bug is **empty `jointNames`**, not a missing wheel.

Cause: `omni.graph.nodes.ConstructArray` v1 `outputs:array` is an unresolved extended `token` (template even authors `custom token`, not `token[]`). Nested as payload, OG often fails to resolve → `jointNames` becomes `()` / `[""]`.

Fix: author `token[] inputs:jointNames = ["left_wheel", "right_wheel"]` on the controller. Drop the ArrayNames connection. Do **not** use joint **indices** (they shift when arms are added/removed).

### USDA: never `delete token[] attr.connect`

```usda
delete token[] inputs:jointNames.connect    # ILLEGAL — no connect target
```

Parser error (`matching … KeywordConnect … Assignment … ConnectValue at ''`). **The whole adapter layer fails to open.** Chassis payload does not load; child prim is `defined=False`, empty children — looks like the mounted robot was “ruined”.

- Do not write `delete <type> <attr>.connect` without `= </Path.outputs:…>`.
- If the child graph no longer has a connection, a stronger-layer **value** is enough.
- Always `Sdf.Layer.FindOrOpen(adapter)` after editing USDA. `Usd.Stage.Open(parent)` will only warn; the child looks missing.

PhysX wheel `damping=1e5` is **not** a MuJoCo actuator gain. Wheels: `stiffness=0` → damping-bias `MjcActuator`; do not copy PhysX damping into `mujoco.usda`.

---

## Troubleshooting matrix

| Symptom | Likely cause | Fix |
|---------|--------------|-----|
| Newton OK, PhysX grippers dead | Wrong/missing PhysX Drive; Side `robotJoints` on inactive joints | Fix `physx.usda` + Side `robotJoints` |
| PhysX open OK, close fails (OCS2) | Adaptive force feedback vs `maxForce` | Lower PhysX `maxForce` below threshold; widen mimic follower limits |
| Only one finger moves | Soft/wrong PhysX mimic (`gearing`/`NF`) | Match `left_physic.usd`: `gearing=-1`, `NF=0` |
| Physics won’t auto-switch after rename | Env still selects old option `"base"` | Update `empty.usda` selections to new names |
| Child Physics stuck | Sticky Physics on mount/env | Remove sticky; push from parent Physics payload or VariantSwitcher |
| Mimic error on EE change | Hot-load during sim | Stop sim before EE variant change |
| Arms freeze under Newton | Side DriveAPI still live | `mujoco.usda` strip Drive; Physics before Side |
| Nested `CreateJoint - no bodies` | Child arm has no Physics default; mount did not sticky Physics (correct) | Parent `Physics/*.usda` `over` child `Physics=physx\|mujoco` |
| Arm joints vanish after mount | Deleted ArticulationRoot APIs on arm `root_joint` | `active = false` only |
| `Invalid DOF name ()` | ConstructArray `jointNames` empty when nested | Author `token[] inputs:jointNames` on the controller |
| Mounted chassis empty / “ruined” | Adapter USDA parse error (often illegal `delete …connect`) | `Sdf.Layer.FindOrOpen` the adapter; remove illegal delete |

---

## Compose smoke test (isaacsim python)

```python
from pxr import Usd, Sdf
assert Sdf.Layer.FindOrOpen(".../payloads/Chassis/tracer_v1.usda")
stage = Usd.Stage.Open(".../env/empty.usda")
root = stage.GetPrimAtPath("/World/ARX_LIFT2S")
root.GetVariantSets().GetVariantSet("Physics").SetVariantSelection("physx")
# Assert nested AC_One_Base / Arm_* / Gripper_* / omnia_150 Physics == "physx"
# Assert left_gripper_joint maxForce / stiffness; left_joint8 gearing / limits
# Print GetPropertyStack for maxForce (physx.usda should beat Side)
# Nested chassis: child prim IsDefined(); ArticulationController jointNames + targetPrim
```

Repeat with `"mujoco"` and confirm Drive stripped / actuators resolve.
