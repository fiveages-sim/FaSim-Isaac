---
name: fasim-robot-mujoco-physics
description: >-
  Ports FaSim-Isaac manipulator/gripper/composite USD assets to MuJoCo/Newton:
  Physics=mujoco, Side left/right MjcActuators, tip/tcp-nested seamless EE mounts,
  VariantSwitcher-friendly child Physics, actuator gravity compensation, and
  composite parents (Galaxea_R1, FiveAges W2, Cobot Magic V1). Use when adding or
  fixing mujoco physics, Newton, Gripper/EE mounts, Side joint renaming / mount
  orientation, parent↔child Sensor/Physics policy, nested chassis ROS, or
  converting robots to match Galaxea_A1X / Marvin_M6_CCS / FiveAges_W2 / Tracer_V1
  patterns.
---

# FaSim Robot MuJoCo / Physics Port

Convert a FaSim-Isaac manipulator, gripper, or composite humanoid to the shared
**MuJoCo / Newton** pattern: root `Physics` + `Side` + `Gripper`/`EE`, seamless EE
prim path, MuJoCo actuators only for existing joints.

**Reference assets (read before copying numbers):**
- `robots/manipulators/Galaxea/Galaxea_A1X/` — arm Physics/Side/actuators (same flange all Sides)
- `robots/manipulators/Galaxea/Galaxea_A1Y/` — same APIs; different joint limits/axes
- `robots/manipulators/Galaxea/Galaxea_A1/` — same APIs; **left/right Gripper mounts use different flange orient**
- `robots/manipulators/Tianji/Marvin_M6_CCS/` — **recommended EE hierarchy** (prim under `link7/tcp`) + EE variant
- `robots/grippers/Jodell/RG75/`, `robots/grippers/ChangingTek/AG2F120S/` — gripper mujoco + Side
- `robots/humanoid/Galaxea_R1/` — torso + SteerChassis mujoco; dual A1 arms
- `robots/humanoid/FiveAges/Gen2/W2/` — torso + Head + Linkhou chassis + dual M6; wheel/steer PD split
- `robots/mobile_manipulator/Agilex/Cobot Magic V1/` — Tracer chassis + dual ARX X5/R5
- `../../../robots/mobile_base/Linkhou/S2_V1/` — four-steer / four-wheel mujoco
- `robots/mobile_base/Agilex/Tracer_V1/` — differential wheels; PhysX velocity drive ≠ MuJoCo actuator
- `robots/manipulators/ARX 6.0/X5/` — Side → EE mount shape (`payloads/EE/`, shared EE prim)

**Do not** blind-copy A1X joint axes / `localPos` / limits / **EE flange quats**. Port
**structure & APIs**; keep target kinematics and Side-specific mount xforms from the
target’s own layers (or legacy binary mounts via `Sdf.Layer.ExportToString`).

## Checklist

```
- [ ] 1. Audit target vs reference gaps
- [ ] 2. Rewrite payloads/Physics/mujoco.usda: MjcJoint only, strip DriveAPI (+ gravcomp)
- [ ] 3. Wire root *.usda Physics=mujoco + Gripper/EE; Physics first in variantSets
- [ ] 4. Text EE mounts nested under tip/tcp (one prim path; preserve Side flange orients)
- [ ] 5. Port Side left/right MuJoCo + Gripper/EE selection
- [ ] 6. Clean robot.usda phantom gripper joints/links
- [ ] 7. If composite: text arm/chassis/head mounts; never bake child Physics
- [ ] 8. Validate composition (no duplicate prim / unresolved actuators / no layer cycle)
```

## 1. Audit

| Piece | Expectation |
|-------|-------------|
| Root variants | `Physics` includes `mujoco`; `Gripper` or `EE` registered on root |
| `Physics/mujoco.usda` | `subLayers = [@./physics.usda@]` + **MjcJointAPI only** (no DriveAPI) + actuators for default + strip Side joint drives |
| EE mounts | Text `.usda`; **nested under tip/tcp** (see §4); same child prim name all Sides |
| Side left/right | Rename joints; `MjcJointAPI` on side joints; side actuators; `active=false` default actuators |
| Isaac rels | Gripper/EE variant prepends `isaac:physics:robotJoints/Links` to **composed EE path** |

## 2. `mujoco.usda` — MjcJoint only (strip PhysX Drive)

Under `Physics=mujoco`, joints must **not** keep PhysX `PhysicsDriveAPI` / drive params. Control is **only** via `MjcJointAPI` + `MjcActuator`.

`delete apiSchemas = ["PhysicsDriveAPI:…"]` alone is **not enough**: values from `physics.usda` (and Side joint `def`s that still author drives) remain as residual `drive:*` attributes. **Block** them in `mujoco.usda`:

```usda
subLayers = [ @./physics.usda@ ]

def "RobotRoot" {
  over "joints" {
    over "jointN" (
      prepend apiSchemas = ["MjcJointAPI"]
      delete apiSchemas = ["PhysicsJointStateAPI:angular", "PhysicsDriveAPI:angular"]
    ) {
      /* strip PhysX drive opinions left by physics.usda */
      float drive:angular:physics:damping = None
      float drive:angular:physics:maxForce = None
      float drive:angular:physics:stiffness = None
      float drive:angular:physics:targetPosition = None
      uniform token drive:angular:physics:type = None
      /* mjc:armature/damping/actuatorfrcrange from maxForce */
      uniform bool mjc:actuatorgravcomp = 1
    }

    /* Side joints: Physics is stronger than Side — strip Drive here too */
    over "left_jointN" (
      prepend apiSchemas = ["MjcJointAPI"]
      delete apiSchemas = ["PhysicsJointStateAPI:angular", "PhysicsDriveAPI:angular", "PhysxJointAPI"]
    ) {
      float drive:angular:physics:damping = None
      float drive:angular:physics:maxForce = None
      float drive:angular:physics:stiffness = None
      float drive:angular:physics:targetPosition = None
      uniform token drive:angular:physics:type = None
    }
    /* same pattern for right_jointN */
  }
  over "linkN" { uniform float mjc:gravcomp = 1 }
  over "root_joint" ( prepend apiSchemas = ["NewtonArticulationRootAPI"] ) { ... }
  def Scope "actuators" {
    def MjcActuator "jointN" { /* default Side only */
      rel mjc:target = </RobotRoot/joints/jointN>
    }
  }
}
```

Rules:
- Joint schemas under mujoco: **`MjcJointAPI` only** for actuation (no `PhysicsDriveAPI` / prefer no living `drive:*`).
- Prismatic grippers: use `PhysicsDriveAPI:linear` / `drive:linear:physics:* = None` analogously.
- Only define actuators for **default** Side joints (`joint1`…); Side actuators stay in `Side/left|right.usda`.
- Do not over non-existent finger/gripper prims on the arm base.
- Map MuJoCo PD from **this** robot’s PhysX (`stiffness`/`damping`/`maxForce`). Galaxea arms often kp=200/kd=10; W2 torso may be 60000/6000; wheels with `stiffness=0` → gain≈0 + damping bias, `ctrlLimited=false`.
- **Do not** copy PhysX wheel `damping=1e5` into MuJoCo actuators (that number is PhysX velocity-drive damping, not MJC `gainPrm`).
- Reference strip pattern: `Marvin_M6_CCS/payloads/Physics/mujoco.usda`.

### Newton gravity compensation

| Attribute | Where | Role |
|-----------|--------|------|
| `mjc:gravcomp = 1` | Rigid **bodies** (torso links, arm `link1…N`) | Actual gravity-compensation force |
| `mjc:actuatorgravcomp = 1` | Actuated **joints** | Route that force through `qfrc_actuator` (with joint force limits) |

Enable both on torso + arm. Default **off** for chassis steer/wheels and gripper fingers unless explicitly requested. Side joints must set `actuatorgravcomp` inside the same `def` as `MjcJointAPI`.

**`physics.usda` must not `subLayers` the robot root** — causes composition cycles when root `Physics=physx|mujoco` payloads it.

## 3. Root asset + VariantSwitcher

```usda
prepend variantSets = ["Physics", "Side", ...]
append variantSets = ["Robot", "Sensor", "Gripper"]  # or EE

variantSet "Physics" = {
  "mujoco" ( prepend payload = @./payloads/Physics/mujoco.usda@ ) {}
  ...
}
```

Prefer **Physics before Side** so Physics opinions win when both contribute.

### Do not sticky child Physics in mounts

Isaac `omni.physics.isaacsimready` **VariantSwitcher** flips every prim with a `Physics` variantSet when the sim backend changes (PhysX→`physx`, Newton→`mujoco`) via the **session layer**.

- Parent arm/humanoid mounts: sticky Side / Sensor / visual only — **never** `variants = { Physics = mujoco }` on child EE/arm.
- If the edit-target/authoring layer already has a Physics selection on that prim, the plugin **skips** it; parent sticky overlays also fight session clears and look like “gripper won’t switch”.
- Validate by **switching the simulator**, not only by changing the parent Physics dropdown (parent Physics alone does not rewrite child sticky assets).

## 4. Seamless EE mounts — tip/tcp nesting (**recommended**)

**Recommended (Marvin M6 CCS):** nest the EE prim under the design tip frame so Stage hierarchy matches the flange and **scene tweaks of gripper install angle stay convenient**:

```
/RobotRoot/linkN/tcp/<EEName>     # tip may be a non-rigid IsaacSite — that is OK
```

Reference: `robots/manipulators/Tianji/Marvin_M6_CCS/payloads/EE/` and `payloads/Side/payloads/EE/`.

Rules for this pattern:
1. Same EE prim name for all Sides (seamless) — never `EE_Left` / `EE_Right` as different prim names.
2. Author `over "linkN" { over "tcp" { def "<EE>" (payload …) { … } } }`.
3. `xformOp:*` are **relative to tcp**; recompute when migrating from a root-sibling mount.
4. `AssemblerFixedJoint`: `body0 = …/tcp` (or tip site), `body1 = …/tcp/<EE>/gripper_base`.
5. Root/Side `isaac:physics:robotJoints/Links` → **full path** `/…/tcp/<EE>`.
6. Sticky gripper `Side`/`side` / flange on the EE asset from the mount; **not** Physics.

| File | EE Side sticky | Arm joint state overs |
|------|----------------|------------------------|
| `*.usda` (default) | `default` | `joint1`… |
| `*_left.usda` | `left` | `left_joint*` + side gripper joints |
| `*_right.usda` | `right` | `right_joint*` + side gripper joints |

Each mount still: `prepend payload` shared EE asset; disable EE `root_joint`; calibrate flange from **target** kinematics / legacy mounts.

**Legacy (A1X/R1):** EE as sibling under robot defaultPrim (`/RobotRoot/Galaxea_G1`) remains valid for existing assets; prefer tip/tcp nesting for **new** arms and when touching EE mounts.

### Side also switches flange orientation

Seamless path ≠ identical transforms. Some arms (notably **Galaxea_A1**) use different `xformOp:orient` / `localRot0` per Side. Restore from legacy mounts; do not copy default/A1X identity into left/right.

## 5. Side left / right

1. `active = false` on default `joint1`….
2. `def PhysicsRevoluteJoint "left_jointN"` (or right) with geometry from **target** Side.
3. Author MuJoCo **inside** that `def` (never a second sibling `over`/`def` same name — **Duplicate prim** kills the Side payload):

```usda
def PhysicsRevoluteJoint "left_joint1" (
  prepend apiSchemas = ["MjcJointAPI"]
  delete apiSchemas = ["PhysicsJointStateAPI:angular", "PhysicsDriveAPI:angular", "PhysxJointAPI"]
) {
  uniform double mjc:armature = 0.01
  uniform bool mjc:actuatorgravcomp = 1
  /* PhysX drives may still be authored here for Physics=physx; mujoco.usda must block them (§2) */
  ...
}
```

4. `over "actuators"`: `active=false` on default actuators; `def MjcActuator "left_jointN"` targeting side joints.
5. `variantSet "Gripper"`/`EE` → left/right mounts, rels to **same** tip/tcp EE path.
6. Prefer putting PhysX `drive:*` for Side joints only where `Physics=physx` wins; under `Physics=mujoco`, §2 overs must strip Drive so only MjcJoint + MjcActuator remain.

Same merge rule for **gripper** Side files (mimic + Mjc in one `def`).

### Gripper assets

Standalone grippers (RG75, AG2F120S, …): own `Physics=mujoco`, Side rename, mimic/Mjc co-authored. If nested Sensor + Side fight, ensure Side opinion can win (variantSet order / sticky strength). No parent Physics overlays onto child grippers.

## 6. Cleanup

- Arm `robot.usda`: only arm joints/links; EE from Gripper/EE payload.
- Remove dead `delete payload` for missing files; drop obsolete binary mounts and phantom EE prim overs (`G1_Left`, root-level EE after tcp migrate).

## 7. Composite parents (dual / embedded arms)

Examples: `Galaxea_R1`; `FiveAges/Gen2/W2` (torso + `Head_V1` + `LinkHou/S2_V1` + dual `Marvin_M6_CCS`); `Cobot Magic V1` (Tracer + dual X5/R5).

| Topic | Rule |
|-------|------|
| Arm Side | Sticky `Side=left` / `right` on each child arm payload |
| Gripper/EE | On child arm (or its default); seamless path including tip/tcp if arm uses that pattern |
| Sensor override | Parent mount may sticky child EE `Sensor` when assembly default differs (e.g. R1 / Cobot Magic → d435) |
| Physics | Parent Physics **payloads** `over` children `Physics=…`; **never** sticky Physics on Chassis/Arm **mounts** (VariantSwitcher) |
| Child no Physics default | X5/R5-style arms: parent **must** push Physics or PhysX `CreateJoint - no bodies` |
| Arm `root_joint` | `active = false` only — do not delete ArticulationRoot APIs |
| Chassis | Drop child ArticulationRoot on the prim that has it + FixedJoint; steer position-PD vs wheel damping-bias |
| Nested ROS | Retarget `targetPrim` → parent root, `chassisPrim` → parent base; author `token[] jointNames` (no ConstructArray v1) |
| Order | Parent `prepend variantSets` starts with `Physics` |
| Validation | `Sdf.Layer.FindOrOpen` adapters; compose full stack; assert child Physics + remapped actuators |

## 8. Validate

For each `Side ∈ {default,left,right}` and `Physics ∈ {physx,mujoco}` with EE selected:

- EE prim under expected **tip/tcp** (or documented legacy sibling path); `AssemblerFixedJoint` + isaac rels match
- Active arm/EE joints match Side naming
- `Physics=mujoco`: actuators resolve; **no live PhysX drive** (`PhysicsDriveAPI` gone; `drive:*` blocked / `Get() is None`); for G-comp check `mjc:actuatorgravcomp` / `mjc:gravcomp`
- `Physics=physx`: DriveAPI + stiffness/damping still present on default and Side joints
- No **Duplicate prim** / **Cycle detected**

Note: without Isaac schema plugins loaded, `GetAppliedSchemas()` may omit `MjcJointAPI` / `NewtonArticulationRootAPI` even when layer opinions exist — trust prim stack + resolved `mjc:target` / blocked `drive:*`.

### Known USD pitfalls

| Symptom | Cause | Fix |
|---------|-------|-----|
| Mujoco still shows stiffness/damping | Only deleted DriveAPI schema; attrs remain from `physics`/Side | In `mujoco.usda` set `drive:* = None` (+ Side joint overs) |
| Side payload fails / EE missing | Duplicate `def`+`over` same name in one layer | Merge into single `def` |
| Gripper UI option gone | Gripper only inside Side payload, not root | Register `Gripper`/`EE` on root |
| Actuator unresolved target | All-Side actuators in shared `mujoco.usda` | Default-only in mujoco; side acts in Side; disable the other set |
| Seamless Side break | Different EE prim names per Side | Unify path under tip/tcp; switch via EE Side |
| Child Physics won’t flip | Parent sticky / authoring-layer Physics opinion | Remove overlay; let VariantSwitcher write session; push from parent Physics payload |
| Gripper “not on tcp” | Root-sibling mount | Nest under tip/tcp; fix rels + relative xform |
| G-comp no effect | Only `actuatorgravcomp` without body `gravcomp` | Set both |
| Composition cycle | `physics.usda` subLayers robot root | Remove that subLayer |
| Left/right flange identical | Copied default flange into all mounts | Restore Side-specific orient/`localRot*` |
| Nested `CreateJoint - no bodies` | Child Physics unset; mount correctly unsticky | Parent `Physics/*.usda` overs child Physics |
| Arm joints gone after weld | Deleted ArticulationRoot on arm `root_joint` | `active = false` only |
| `Invalid DOF name ()` | Nested ConstructArray `jointNames` empty | Author `token[] inputs:jointNames` on controller |
| Mounted chassis empty | Adapter USDA parse fail (`delete …connect` without target) | Open adapter with `Sdf.Layer.FindOrOpen`; fix USDA |
| MJC wheels explode / overdamped | Copied PhysX `damping=1e5` into actuators | `stiffness=0` → small gain + damping-bias only |

## Extra detail

- Mount/actuator templates, A1 flange notes, R1/W2/Cobot Magic chassis: [reference.md](reference.md)
- Nested chassis ROS / illegal USDA `delete …connect`: companion `USDA-OCS2-PhysX-Mujoco` [reference.md](../USDA-OCS2-PhysX-Mujoco/reference.md)
