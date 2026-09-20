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

**AgileX Piper gripper** (prismatic `axis=Z`, both fingers often share world axis `(0,-1,0)`): `gearing=-1` + `localRot1` Rz180 makes **both pads travel the same way**. Validated: PhysX `physxMimicJoint:rotZ:gearing = 1`, Newton `newton:mimicCoef1 = -1`. Check world axes before copying ARX `gearing=-1`.

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

### PhysX wheel Drive `type`

Steer and wheel **do not share** Drive type. Authoritative chassis: `robots/mobile_base/Linkhou/S2_V2/` (Ranger Mini matches after port).

```usda
over "fl_steer_joint" {
    float drive:angular:physics:stiffness = 60000
    float drive:angular:physics:damping = 6000
    uniform token drive:angular:physics:type = "force"          /* position */
}
over "fl_wheel_joint" {
    float drive:angular:physics:stiffness = 0
    float drive:angular:physics:damping = 100000
    float drive:angular:physics:targetPosition = 0
    uniform token drive:angular:physics:type = "acceleration"   /* velocity */
}
```

| Joint | `type` | Why |
|-------|--------|-----|
| Steer | `force` + `targetPosition` | PD on module yaw |
| Wheel | **`acceleration`** + `targetVelocity` | `damping` is a velocity tracking gain |

`force` + `damping=1e5` on wheels → `τ = D·(ω*−ω)` (huge torque) → four wheels fight → chassis jitters / surges. `cmd_vel` script: `linearGain ≈ 1/r` (Linkhou S2 `r=0.07` → **14.28**; Ranger Mini `r=0.09` → **11.262**), plus `(θ,v) ≡ (θ+π,−v)` continuity. Parent `joint_command` ArticulationController must **not** connect velocity/effort (would overwrite `cmd_vel` wheel targets).

### 4WS cmd_vel (Ranger Mini / Split Aloha)

IK: `vx_i = vx − ω y_i`, `vy_i = vy + ω x_i`, `θ = atan2(vy_i, vx_i)`, `v = ‖(vx_i, vy_i)‖`. Wheel order must match authored `jointNames` (Ranger Mini: fl, rl, rr, fr).

**`vy` + `ω` at once:** the four target angles jump 60°–120° apart while wheel `acceleration` drive applies immediately → scrub, left-right rock, “卡住”. Gate **all four** wheel speeds by the **worst** steer error (not per-wheel): ≥40° → 0, ≤12° → 1. Keep a rate-limited steer estimate at `maxJointVelocity` (script `_STEER_SLEW` **must match** PhysX `physxJoint:maxJointVelocity`). Pick `(θ, v)` vs `(θ+π, −v)` against the **estimate**, not last command (command history is 1-frame). Sign-flip scale (~0.35) still applies.

Do **not** add `IsaacReadJointState` on the nested Ranger graph without retargeting to the **merged** articulation root (`Split_Aloha/base`); the lag model is portable standalone + nested.

### Steer PD (do not copy Linkhou onto Ranger Mini)

| | Linkhou S2 (stiff) | Ranger Mini (validated 2026-09) |
|--|--------------------|----------------------------------|
| type | `force` | `force` |
| K / D | 60000 / 6000 | **1500 / 300** (`D ≈ 0.2 K`) |
| maxForce | large | **120** Nm (40 Nm saturates on tire scrub) |
| maxJointVelocity | — | **180** °/s (90 °/s too slow → stuck; 360 too snappy) |
| MuJoCo | map K/D/F | `gainPrm=[K]`, `biasPrm=[0,-K,-D]`, `forceRange=±F` |

Symptoms: small-angle overshoot → lower K (not only raise D). Steer late / chassis rocks on crab+yaw → raise `maxForce`/`maxJointVelocity` and keep the align-gate; do not return to K=60000.

### Pitch nod vs roll (tall 4WS + upper box)

Narrow track + high COM → drive pitch (about Y / `Iyy`). Lower **upper-box COM z** and add **low chassis ballast**; raising box `Iyy` stores energy and often still nods. Left-right rock after soft steer is usually **4WS scrub**, not missing `Ixx`. Lift prismatic-Z is not a pitch DOF.

### Nested dexhand self-col (Split Aloha / Piper Revo)

Same as W2/Luna: parent art-root self-col **ON**; `NonDexHand` mutes body/chassis/arm; EE `dexhand_self_collision.usda` excludes hand roots; grippers stay in the group.

Split Aloha: `payloads/Physics/body_self_collision_mute.usda` (physx subLayer) includes `base` / `lifting_link` / `Ranger_Mini` / both `…/Piper`; excludes `…/Piper/link6/tcp/Revo1|Revo2`; **cross-filter** parent ↔ nested `Piper/CollisionGroups/NonDexHand` (else arm hull vs box chatter). Newton: `newton:selfCollisionEnabled=1` + **append references** `body_self_collision_mute_newton.usda` on root `Physics=mujoco` (not mujoco subLayer; **no Piper paths** — races Arm payloads). Arms: Piper `arm_self_collision_mute_newton`. Hands ungrouped.

### Optical USD Camera (no ROS until asked)

USD Camera looks **−Z**; ROS/optical `camera_link` looks **+Z** → child Camera `orient = (0, 1, 0, 0)` (Rx180). Sibling of the visual mesh — **not** under dabai’s compensation xform (`orient` + `rotateX:unitsResolve`).

Aperture from FOV (`f` = Isaac default 18.147562): `aperture = 2 f tan(fov/2)`. Split Aloha lift dabai RGB **16:9 H86° V55° D93.5°±3°** → `horizontalAperture=33.84575` `verticalAperture=18.89405`. **4:3 H64° V55°** is a horizontal crop of the same lens — do not author a second Camera.

### Piper Revo1 / Revo2 EE (tcp)

`AssemblerFixedJoint` `localPos0/localRot0` **=** mount `translate/orient`. Native: Revo2 palm **+Z**, Revo1 palm **+Y**.

| EE / Side | Mount (T z=0.0105) | Notes |
|-----------|--------------------|--------|
| Revo2 L/R/default | `Rz(−90)` `(0.707, 0, 0, −0.707)` | Palm inward (tcp −Y left / +Y right after X-mirror), **not** palm-down |
| Revo1 L/default | `Rx(+90)` `(0.707, 0.707, 0, 0)` | Native-to-Revo2 align ∘ `Rz(−90)` |
| Revo1 right | `Rx(+90)` then `Rz(+180)` `(0, 0, 0.707, 0.707)` | Extra spin after X-mirror |

Do not copy Revo2 identity onto Revo1 (stays palm-down). Flange visual is a **tcp sibling** so it does not inherit hand rotation.

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
| Chassis jitters / wheels fight | Wheel Drive `type=force` + `damping=1e5` | Set wheels to `acceleration` (steer stays `force`); `linearGain≈1/r` |
| 4WS crab+yaw rocks / stuck | Wheel vel before steers arrive; `maxForce`/`maxVel` too low | Align-gate (min of 4); match slew to `maxJointVelocity`; F≈120, ωmax≈180°/s |
| Steer small-angle overshoot | Copied Linkhou K=60000 | Ranger Mini K=1500 D=300 (`D≈0.2 K`) |
| Pitch nod on accel | High upper COM, short wheelbase | Lower box COM z + chassis ballast; do not just raise `Iyy` |
| Nested Revo fingers penetrate | Parent self-col off or no EE excludes | Parent self-col ON + NonDexHand + `dexhand_self_collision` Revo excludes |
| Camera looks backward / through housing | Camera under dabai xform, or no Rx180 | Sibling of mesh on optical link; `orient=(0,1,0,0)` |
| Revo1 still palm-down | Identity / old Rx90·Rz90 only | Revo1 `Rx(+90)`; right + `Rz(+180)` |
| Piper gripper both pads same side | Copied ARX `gearing=-1` on co-axial fingers | PhysX `gearing=+1`, Newton `mimicCoef1=-1` |

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
