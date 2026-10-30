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

| Pattern | ArtRoot on | `targetPrim` / CM | Synth joint scope | When |
|---------|------------|------------------|-------------------|------|
| **W2** | `/FiveAges_W2/base_link` | same (createArticulationView) | node expands to `/FiveAges_W2` | DOFs under sibling `joints/` + arms; nested Head/Torso/EE ArtRoot must be deleted |
| **G1D composite** | `/G1D` (robot Xform) | `/G1D` | same | Stub hub `/G1D/base_link` is Assembler-only — ArtRoot there → `Pattern did not match` |

### W2 / W2R: `Robot` variant == description stem

Same USD asset [`FiveAges/Gen2/W2`](robots/humanoid/FiveAges/Gen2/W2/). Product SKU is the
`Robot` variant name (Galaxea-style arm pin + emit robot, **no remap table**):

| `Robot=` | Arms pinned | emit `--robot` | Env |
|----------|-------------|----------------|-----|
| `fiveages_w2` | `M6_CCS` | `fiveages_w2` | `env/fiveages.usda`, `env/xhand1.usda` (EE=Xhand1) |
| `fiveages_w2r` | `AR5_CCS_V2` | `fiveages_w2r` | `env/fiveages_w2r.usda` |
| `robot` (legacy) | same as w2 | node aliases → `fiveages_w2` | — |
| `none` | (manual Arm_*) | error if `inputs:robot` empty | — |

`ROS=fiveages` → [`payloads/ROS/fiveages.usda`](robots/humanoid/FiveAges/Gen2/W2/payloads/ROS/fiveages.usda):
CM + `/clock`; **no** body JointState topic bridge; pushes `Linkhou_S2` `ROS=enable`
for chassis cmd_vel/odom. `inputs:robot=""` → node walks ArtRoot ancestors for
`Robot` (ArtRoot is `base_link`, variant lives on `/FiveAges_W2`).

Rules:

- Sole **active** ArtRoot on the whole robot; strip Chassis/Lift/Body/Arms importer roots.
  EE uses §2.1 (`root_joint active=false`), not `delete` ArtRoot on the gripper base.
- Floating hub (`/G1D/base_link`): `RigidBody` + real mass/inertia (tiny mass rattles); **not** ArtRoot on G1D-style composites.
- Physics backend ↔ `Physics` variant；`ROS=fiveages` / OCS2 正交。`IsaacSimSystem` = **PhysX + Newton**。
  - **Asset root** may default `Physics=physx`（G1D：空选择 → 无 ArtRoot → Assembler 双臂散架）。
  - **Mount / env 禁止 sticky `Physics=`**；子件由父 `payloads/Physics/{physx,mujoco}.usda` push。
  - Newton：改选 `Physics=mujoco`（或清掉根默认后靠 VariantSwitcher）；卡住则 Reload Stage。勿用 env pin 绑死。
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
- [ ] mode = full_body | split_body | demo
- [ ] robot = unitree_g1d / … OR empty + Robot=fiveages_w2|fiveages_w2r (name == package stem)
- [ ] enableArmsTargetManager; type empty → EE / EE_Left|Right / Left_Arm|Right_Arm EE → 运控 key (PascalCase→lower except CR5 AG2F*)
- [ ] Physics=physx pinned; ROS=fiveages; PhysX-only PhysicsScene; /clock if useSimTime
- [ ] W2 Xhand/Dex EE: mount deletes nested ArtRoot schemas (not only `active=false`)
- [ ] W2: Linkhou child ROS=enable for DiffDrive; no body JointState↔ArticulationController bridge
- [ ] target_manager.yaml present for that description package (src + install)
- [ ] Chassis/EE Assembler localPos matches USD TF; EE body0 is RigidBody
- [ ] After Play: no “link not unique”, no tensors invalidated, ATM logs correct frames
```

### Multi-robot namespace (dual G1D / …)

Same string on **all** of: `ROS2ControlLaunch.inputs:namespace`, DiffDrive/odom TF
`nodeNamespace`, ATM `__ns` (node auto-spawns with `__ns:=/<ns>`).

| Rule | Detail |
|------|--------|
| Business topics | Relative only — never leading `/`. Robot-level: `fsm_command`/`cmd_vel`/`robot_description`. Controller-private: keep name → `controller_name + "/waist_*"` / `"/target_command"` (→ `/<ns>/<ctrl>/…`) |
| Global OK | `/clock`, `/parameter_events`, `/diagnostics`; VR headset `/teleop/*` may stay global |
| TF | Per-robot `/<ns>/tf` (+ static). Do **not** merge fleets onto one `/tf`. Frame names may stay `base_link` |
| `world→odom` | **Play-first latch** of chassis world pose (`LatchWorld2Odom`). Spawn G1D at **z=0**. Not every-frame current base. **Must re-latch on Stop→Play** via simTime rewind / frame drop / (`odom≈0` + hub jumped >10cm); settle ~5 ticks. ScriptNode `setup()` does not re-run — without this, TF sticks after moving ArtRoot while Stopped. Reload ROS payload after editing |
| `odom→*` | `IsaacComputeOdometry` **relative** displacement from Play (`/odom` + `odom→base_footprint`) |
| WBC `tf_topic` | Must be relative `tf` (→ `/<ns>/tf`), **not** absolute `/tf`. Otherwise activate probe misses Isaac TF and disables unlocked-base planning |
| Chassis topics | Payload `topicName` = relative `cmd_vel` / `odom` (+ `nodeNamespace`) |
| ATM | Node `arms_target_manager_<ns>` under `/<ns>`; spawn also remaps `/tf:=tf` and `/tf_static:=tf_static` (TransformListener ignores `__ns` for absolute `/tf`) |
| RViz / tf2 CLI | Default listens `/tf`. Use `ros2 run tf2_ros tf2_echo world base_link --ros-args -r /tf:=/g1d/tf -r /tf_static:=/g1d/tf_static` (or RViz TF topic remaps). Empty `/tf` with ns enabled ≠ missing Isaac TF |
| Helper nodes | Same short name under ns prefix: `/<ns>/fasim_robot_description`, `/<ns>/fasim_ros2_control_launch` |

Dual room acceptance (`environment/.../G1D/dual/room.usda`, ATM off OK):

```
- [ ] CM / DiffDrive / odom TF nodeNamespace == ROS2ControlLaunch.namespace (`g1d`, `g1d_01`)
- [ ] `/g1d/cmd_vel` and `/g1d_01/cmd_vel` both exist; root `/cmd_vel` gone (or only unrelated nodes)
- [ ] `/g1d/odom` and `/g1d_01/odom`; `/g1d/tf` + `/g1d_01/tf` (no fleet merge on global `/tf`)
- [ ] Play: `tf2_echo world base_link` on `/g1d/tf` ≈ stage G1D pose (not origin); `/g1d/odom` pose still ~0 and accumulates; `world→odom` stays latched
- [ ] `/g1d` vs `/g1d_01` have distinct `world→odom` (different spawn); Stop→Play relatch
- [ ] No global `/fsm_command` `/fsm_state` `/ocs2_wbc_controller/*` `/*_gripper_*` business leaks
- [ ] Controllers under `/g1d/...` and `/g1d_01/...` (relative/`~/` expanded)
- [ ] Single-robot `namespace=""` still uses root paths (`/cmd_vel`, `/fsm_command`, …)
```

## 6. Common errors → fix

| Log / symptom | Fix |
|---------------|-----|
| `Pattern '…/base_link' did not match any articulations` | (1) `Physics` 变体与当前引擎不一致（Switcher 未切到 / sticky 挡住）— 勿 env pin，清 mount sticky，Reload。 (2) Nested EE ArtRoot: mount `active=false` + `delete apiSchemas`。 (3) Composite ArtRoot 在 robot Xform — retarget `targetPrim` |
| JSB `No state interfaces found to publish` | Synth HW empty: W2 ArtRoot=`base_link` but DOFs are siblings — node must expand synth scope to robot Xform; also mute nested ArtRoots so `discover_articulations` sees exactly one root |
| `'/World' contains N ArticulationRootAPI` (dual robot) | Synth scope must not expand ArtRoot→World when siblings are other robots; each CM `targetPrim` stays on its robot ArtRoot |
| `disjointed body transforms` / whole robot snaps | Fix Assembler `localPos0` to match link offset |
| MimicJoint / nested ArtRoot / Dex1 | ArtRoot on `root_joint`; mount `active=false` + `delete apiSchemas`; weld to tcp/wrist |
| Dual G1D arms scatter (Dex3) | (1) Dex3 `Side=right` must not re-payload Hand/right Physics (double ArtRoot). (2) Mount: `root_joint` active=false + delete ArtRoot on palm. (3) Place `G1D_01` — missing TF → origin depenetration kills both arts |
| JSB `left_gripper_joint/position` not available (Dex3) | `dex3` must be in `eef_control_registry` as **hand** (unitree_dex `dex3.side.yaml`). Unregistered → gripper template → JSB claims `left_gripper_joint` while Isaac HW has `left_hand_*` |
| Dual EE type keys | G1D `EE_Left`/`EE_Right` → emit `left_type`/`right_type`; `dex1`=gripper, `dex3`=hand |
| Global `/fsm_*` `/cmd_vel` `/ocs2_wbc_controller/*` with dual ns | Controllers/ATM used absolute `/…`; must be relative (controller-private keep `ctrl/…`). Rebuild arms packages; ATM spawn must pass `__ns` |
| `link 'base_link' is not unique` | Need `_dedupe_urdf_duplicate_names` (reload node) |
| ATM VR “weird” / no teleop | Add/fix `target_manager.yaml` frames; enable_vr |
| Continuous track offset on mobile WBC | `control_base_frame`+`marker` must be `world`; vr_follow stays footprint |
| tensors invalidated mid-run | Don’t flip EE/Physics variants on Play; fix snap first |
| G1D 进场景后再选 Physics → 双臂掉落 | mount/env sticky Physics 与 session 打架 → Assembler `disjointed`。G1D 已去 sticky；子件由父 Physics payload push；换引擎卡住 Reload |
| Newton 下 fiveages 异常 | `IsaacSimSystem` 支持 Newton；先确认 `Physics=mujoco` 与引擎一致、资产 mujoco 轨完整。勿用 env pin PhysX 绕过 |
| `get_ros2_control_robot_description returned empty` (G1D / caster xacro) | Post-guard partial AMENT + stub `/opt/ros/.../component_models` shadows ros2_ws. `_prepare_host_ros_env` must clear AMENT/COLCON/CMAKE/PYTHONPATH before `source setup.bash` so overlay wins |
| `dlopen … libtracetools.so: cannot open shared object file` / ocs2_wbc load fail | `./run.sh` must use `FASIM_FULL_HOST_ROS=1` (keep `/opt/ros` on LD) + exclude pink. Do **not** strip underlay LD. `ensure_isaac_ros2_library_path` keeps ws `lib/` before `/opt/ros` so apt `libocs2_*` cannot shadow |
| apt `libocs2_*.so` shadows workspace (pinocchio SONAME mismatch) | Overlay-first LD; or remove apt ocs2 packages. Workspace sourced ⇒ its `install/*/lib` must precede `/opt/ros/*/lib` |
| W2 `inputs:robot` empty + `Robot=none` | Set `Robot=fiveages_w2` or `fiveages_w2r`, or hardcode `inputs:robot` |

## Related

- `USDA-OCS2-PhysX-Mujoco` — articulation / Physics variants / DiffDrive
- `fasim-dexhand-asset` — EE Assembler + PhysX hand baseline
- robot_common_launch: `prepare_arms_target_manager_parameters`, `parse_task_info`
