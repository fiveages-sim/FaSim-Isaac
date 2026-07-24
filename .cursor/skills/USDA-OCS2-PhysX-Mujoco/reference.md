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

`FaSim-Isaac/robots/manipulators/ARX/ARX5_Gripper_2025/configuration/left_physic.usd` (used by old `ARX_Lift2S`):

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
| One root | Chassis `PhysicsArticulationRootAPI` |
| Child roots | Deleted on assembled child `base_link` / gripper `root_joint` |
| `robotJoints` | Includes arms + **active** gripper joints under merged tree |
| Controller target | OmniGraph `ArticulationController` → chassis `base_link` |
| Joint names | Match URDF / OCS2 (`left_gripper_joint`, `left_joint1`, …) |

---

## Troubleshooting matrix

| Symptom | Likely cause | Fix |
|---------|--------------|-----|
| Newton OK, PhysX grippers dead | Wrong/missing PhysX Drive; Side `robotJoints` on inactive joints | Fix `physx.usda` + Side `robotJoints` |
| PhysX open OK, close fails (OCS2) | Adaptive force feedback vs `maxForce` | Lower PhysX `maxForce` below threshold; widen mimic follower limits |
| Only one finger moves | Soft/wrong PhysX mimic (`gearing`/`NF`) | Match `left_physic.usd`: `gearing=-1`, `NF=0` |
| Physics won’t auto-switch after rename | Env still selects old option `"base"` | Update `empty.usda` selections to new names |
| Child Physics stuck | Sticky Physics on mount/env | Remove sticky; use nested overs or VariantSwitcher only |
| Mimic error on EE change | Hot-load during sim | Stop sim before EE variant change |
| Arms freeze under Newton | Side DriveAPI still live | `mujoco.usda` strip Drive; Physics before Side |

---

## Compose smoke test (isaacsim python)

```python
from pxr import Usd
stage = Usd.Stage.Open(".../env/empty.usda")
root = stage.GetPrimAtPath("/World/ARX_LIFT2S")
root.GetVariantSets().GetVariantSet("Physics").SetVariantSelection("physx")
# Assert nested AC_One_Base / Arm_* / Gripper_* / omnia_150 Physics == "physx"
# Assert left_gripper_joint maxForce / stiffness; left_joint8 gearing / limits
# Print GetPropertyStack for maxForce (physx.usda should beat Side)
```

Repeat with `"mujoco"` and confirm Drive stripped / actuators resolve.
