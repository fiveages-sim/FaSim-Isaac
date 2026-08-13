# Reference: MuJoCo / Gripper port patterns

## Directory layout (Galaxea-style)

```
RobotName/
  RobotName.usda                 # root variants
  payloads/
    base.usda, robot.usda, ...
    Physics/
      none.usda | physics.usda | physx.usda | mujoco.usda
    Side/
      default.usda | left.usda | right.usda
      payloads/Gripper/          # or EE/
        g1.usda | g1_left.usda | g1_right.usda
```

X5 uses `EE` + `payloads/EE/{2023,2025,left_*,right_*}.usda` with the same “one prim path” idea.

Galaxea_R1 (composite) layout extras:

```
Galaxea_R1/
  payloads/Physics/mujoco.usda          # torso actuators
  payloads/Chassis/steer_chassis.usda   # SteerChassis mount (variant Chassis=SteerChassis)
  payloads/Robot/r1.usda
  payloads/Robot/r1 pro.usda
  payloads/Robot/payloads/Arm_Left/galaxea_a1.usda    # R1 left A1 (Side=left, Sensor=d435)
  payloads/Robot/payloads/Arm_Right/galaxea_a1.usda   # R1 right A1 (Side=right, Sensor=d435)
  payloads/Robot/payloads/Arms/a2_arms.usda           # R1 Pro A2 dual-arm mount
  components/SteerChassis/payloads/Physics/mujoco.usda
```

Prefer payload filenames that match meaning (`galaxea_a1.usda`, `a2_arms.usda`, `steer_chassis.usda`) over generic `base.usda`. Variant option names should match (`Arm_Left=galaxea_a1`, `Arms=a2_arms`).

## Actuator block template

```usda
def MjcActuator "joint1"
{
    uniform token mjc:gainType = "fixed"
    uniform token mjc:biasType = "affine"
    uniform double[] mjc:gainPrm = [200, 0, 0, 0, 0, 0, 0, 0, 0, 0]
    uniform double[] mjc:biasPrm = [0, -200, -10, 0, 0, 0, 0, 0, 0, 0]
    uniform token mjc:forceLimited = "true"
    uniform double mjc:forceRange:min = -27
    uniform double mjc:forceRange:max = 27
    uniform token mjc:ctrlLimited = "true"
    uniform double mjc:ctrlRange:min = -165.00009
    uniform double mjc:ctrlRange:max = 165.00009
    rel mjc:target = </RobotRoot/joints/joint1>
}
```

Map from PhysX joint:
- `forceRange` ↔ `drive:*:physics:maxForce`
- `ctrlRange` ↔ `physics:lowerLimit` / `upperLimit`
- Gripper prismatic: often `gainPrm=100`, `biasPrm=[0,-100,-10,...]`, ctrl in meters

## Gripper mount skeleton

```usda
#usda 1.0
(
    defaultPrim = "RobotRoot"
)

def Xform "RobotRoot"
{
    def "Galaxea_G1" (
        prepend payload = @../../../../../Galaxea_G1/Galaxea_G1.usda@
        variants = {
            string Sensor = "d405"
            string Side = "left"   # default | left | right
        }
    )
    {
        double3 xformOp:translate = (...)
        quatd xformOp:orient = (...)   # may differ per Side — see Galaxea_A1
        uniform token[] xformOpOrder = ["xformOp:translate", "xformOp:orient", "xformOp:scale"]

        over "root_joint" ( active = false ) {}

        over "gripper_base"
        {
            def PhysicsFixedJoint "AssemblerFixedJoint"
            {
                rel physics:body0 = </RobotRoot/link6>
                rel physics:body1 = </RobotRoot/Galaxea_G1/gripper_base>
                point3f physics:localPos0 = (...)
                point3f physics:localPos1 = (...)
                quatf physics:localRot0 = (...)  # Side-specific on A1
                quatf physics:localRot1 = (1, 0, 0, 0)
            }
        }

        over "joints"
        {
            over "left_gripper_joint" { /* state zeros; name must match G1 Side */ }
            over "left_gripper_joint2" { }
        }
    }

    over "joints"
    {
        over "left_joint1" { float state:angular:physics:position = 0 ... }
        ...
    }
}
```

## Side Gripper variant (minimal)

```usda
variantSet "Gripper" = {
    "None" {}
    "galaxea_g1" (
        prepend payload = @payloads/Gripper/g1_left.usda@
    ) {
        prepend rel isaac:physics:robotJoints = </RobotRoot/Galaxea_G1>
        prepend rel isaac:physics:robotLinks = </RobotRoot/Galaxea_G1>
    }
}
```

## Composition validation snippet

Requires Isaac / `omni.usd.libs` Python on `PYTHONPATH` / `LD_LIBRARY_PATH`:

```python
from pxr import Usd

stage = Usd.Stage.Open("RobotName.usda")
root = stage.GetPrimAtPath("/RobotRoot")
root.GetVariantSet("Side").SetVariantSelection("left")
root.GetVariantSet("Gripper").SetVariantSelection("galaxea_g1")
root.GetVariantSet("Physics").SetVariantSelection("mujoco")

ee = stage.GetPrimAtPath("/RobotRoot/Galaxea_G1")
assert ee.IsValid()
acts = stage.GetPrimAtPath("/RobotRoot/actuators")
for c in acts.GetChildren():
    if c.IsActive():
        print(c.GetName(), c.GetRelationship("mjc:target").GetTargets())
```

Binary crate inspection without flatten:

```python
from pxr import Sdf
print(Sdf.Layer.FindOrOpen("path/to/g1.usd").ExportToString())
```

## Galaxea_A1 flange orients (do not flatten to default)

Same EE prim `/Galaxea_A1/Galaxea_G1`; Side file chooses orient/`localRot0` (from legacy crates):

| Side | `xformOp:orient` (approx) | `localRot0` |
|------|---------------------------|-------------|
| default | identity `(1,0,0,0)` | `(-0.707, 0.707, 0, 0)` |
| left | +90° X `(0.707, 0.707, 0, 0)` | identity |
| right | −90° X `(0.707, -0.707, 0, 0)` | `(0, 1, 0, 0)` (180° X) |

A1X/A1Y can keep one flange for all Sides; A1 cannot.

## Parent overrides child Gripper Sensor (R1)

Standalone A1 mounts often `Sensor=d405`. R1 arm mounts force d435 on the child EE without changing A1 assets:

```usda
over "Galaxea_G1" (
    variants = {
        string Sensor = "d435"
    }
) {
    over "joints" { /* side gripper joint zeros */ }
}
```

## Nested arm mount sketch (R1)

```usda
def "Galaxea_A1_Left" (
    prepend payload = @.../Galaxea_A1/Galaxea_A1.usda@
    variants = {
        string Gripper = "galaxea_g1"
        string Side = "left"
        string Sensor = "none"   # arm body sensors; EE camera is on Galaxea_G1
        ...
    }
) {
    over "root_joint" ( active = false ) {}
    over "base_link" {
        def PhysicsFixedJoint "AssemblerFixedJoint" { /* torso_link4 ↔ A1 base */ }
    }
    over "Galaxea_G1" ( variants = { string Sensor = "d435" } ) { ... }
}
```

Prim rename (`Galaxea_A1` → `Galaxea_A1_Left`) remaps absolute paths under the child; actuators under mujoco still resolve when child `Physics=mujoco` is selected.

## Naming cheat sheet

| Side | Arm joints | G1 joints (Galaxea) |
|------|------------|---------------------|
| default | `joint1`…`joint6` | `gripper_joint`, `gripper_joint2` |
| left | `left_joint1`… | `left_gripper_joint`, `left_gripper_joint2` |
| right | `right_joint1`… | `right_gripper_joint`, `right_gripper_joint2` |

## What Isaac VariantSwitcher expects

- Root may omit a sticky `Physics=` opinion if auto-switching from Newton is required (session layer).
- Child EE / child arm Physics is usually flipped by VariantSwitcher when parent uses Newton/MuJoCo — avoid stacking conflicting child Physics selections in parent Physics variants or in Arm mounts unless tested.
- Prefer sticky `Robot=` / `Chassis=` on composites so envs that only set a subset of variants still compose arms.

## Nested chassis (Tracer / Cobot Magic)

Match Galaxea_R1: Chassis adapter payloads the shared base, welds `AssemblerFixedJoint`, zeros wheel joint states. Sticky `ROS=enable` + Graph retarget on the mount or parent root `over`; **never** sticky `Physics` on the mount.

- ArticulationRoot: delete on the prim that has it (Tracer: child root; SteerChassis: `base_link`). Arms: `root_joint` `active=false` only.
- Parent Physics payloads must `over` Tracer / X5 / R5 / nested grippers with matching `Physics=` (X5/R5 have no default).
- Nested `ArticulationController`: author `token[] inputs:jointNames`; ConstructArray v1 output does not resolve under payload → `Invalid DOF name ()`.
- **Illegal USDA:** `delete token[] inputs:jointNames.connect` (no `= </…>`) fails the **entire** adapter parse; mounted child is empty. Always `Sdf.Layer.FindOrOpen` the adapter.
- PhysX wheel velocity drive (`stiffness=0`, `damping=1e5`) stays in `physx.usda`. MuJoCo wheels: damping-bias actuators only — do not copy `1e5`.
