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
| Host helpers | `fasim/ros2/nodes/host_ros.py` (setup.bash resolve, env reset, process group) |
| Nav OGN | `.../OgnNavigationLaunch.ogn` + `OgnNavigationLaunch.py` (Nav2 on Play) |
| Robot YAML | `<robot>_description/config/ros2_control/` + `config/ocs2/` |
| Env wiring | `robots/**/payloads/ROS/fiveages.usda` → Graph `ROS2ControlLaunch` / `ROS_Navigation` |

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

`emit_control` injects `recompile_libraries: false` on the main OCS2 controller so Isaac reuses CppAD caches under `/tmp/ocs2_ros2/...` (set `true` or clear the cache after model/constraint changes).

## 4. ATM / VR frames (`target_manager.yaml`)

ATM is a **host** process (`use_sim_time=false`, nice=10). CM stays on sim time + `/clock`.
The `ROS=fiveages` payload sets `ReadSimTime.inputs:resetOnStop=1`, so `/clock`
restarts from 0 on Stop→Play (the topic-bridge variant `ROS=enable` keeps `0`).

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

## 4.1 `/clock` must be physics-stepped, not render-frame (OnPlaybackTick)

In-process CM (`use_sim_time=1`) reads its clock from `/clock`. If `/clock` is
published from `OnPlaybackTick`, controller trajectory time (linear `percent_`
and moveJ/doubles `real_t`) advances at the **render-frame** cadence, not the
physics rate — duration lies.

**Symptom (W2, PhysX 200 Hz / update_rate 200):** CM `update()` reports
`period` jittering `0 / 0.015 / 0.020 s` instead of a steady `0.005 s`. moveJ
`real_t` then races at ~3× sim time, so a `home_duration=5` finishes in ~1.5
wall seconds. `period` always equals the sim-time delta (`diff=0`), so the bug
is the **clock source**, not period accumulation.

**Root cause:** `OnPlaybackTick` fires per render frame (~60–70 Hz, uneven).
Between two frames the physics steps 3–4× (0.015–0.020 sim s), and sometimes 0×
when a frame is fast. `PublishClock` bound to that tick forwards those jumps
straight into the CM clock.

**Fix (applied to all fiveages payloads):** publish `/clock` from the physics
step in a **dedicated on-demand graph** `ROS_Clock`, sibling to `ROS_Control`:

- `OnPhysicsStep` (`isaacsim.core.nodes.OnPhysicsStep`) only triggers in an
  **on-demand (push)** graph. The `ROS_Control` graph is `Automatic` and must
  stay that way for `ROS2ControlLaunch` / Nav, so the clock cannot live there.
- Graph attrs that make it actually on-demand — **`pipelineStage =
  "pipelineStageOnDemand"`** is the decisive one (just
  `evaluationMode="OnDemand"` is not enough → `Physics OnSimulationStep node
  detected in a non on-demand Graph`). Mirror the working
  `robots/dexhands/Wuji/Hand2/payloads/Sensor/contact_force.usda` graph:
  `evaluationMode="OnDemand"`, `evaluator:type="push"`,
  `pipelineStage="pipelineStageOnDemand"`, and `prepend apiSchemas =
  ["NodeGraphNodeAPI"]` on each node.
- `OnPhysicsStep.step` → `PublishClock.execIn`; `ReadSimTime
  (resetOnStop=1)` → `PublishClock.timeStamp`. Remove the old
  `ReadSimTime`/`PublishClock` from `ROS_Control` so there is exactly one
  `/clock` publisher.
- Per-robot connection paths use that robot's root prim (`</CR5/...`,
  `</G1D/...`, `</Zerith_H1/...`, `</Piper*/...`, `</FiveAges_W2/...`).

**Do NOT blanket-delete every `ReadSimTime`.** Only the `/clock` one in
`ROS_Control` is relocated. Some robots carry a **second** `ReadSimTime` inside
their own `ROS_Odometry` graph that feeds odom/TF `timeStamp` — G1D is one
(`chassisFrameId=base_footprint`, `world→odom→base_footprint` live in
`payloads/ROS/fiveages.usda`, not a separate chassis payload). A regex that
drops `def OmniGraphNode "ReadSimTime"` everywhere orphaned G1D's 5 odom/TF
publishers → stamp 0 → `Timestamp is invalid. Timestamp 0 will be neglected`
for `world`/`odom`/`base_footprint`, while arm/dex TF (CM clock) stayed fine.
Tell-tale: `/odom` stamp stuck at `sec:0 nanosec:0` but pose values still move.
Fix = restore that graph's own `ReadSimTime` (`resetOnStop=1`). After any bulk
edit, grep that every `Graph/<X>/ReadSimTime.outputs` reference still has a
matching node definition.

**After the fix:** CM `period` is a steady `0.005 s` (`ratio=1.0000`), moveJ
`real_t` advances ≈1:1 with sim time, and `home_duration` tracks sim seconds
(wall ≈ sim / real-time-factor). Verify with `ros2 topic hz /clock` ≈ physics
rate and a home switch.

Covered payloads (all fixed): W2, Zerith H1, G1D, AgileX Piper / Piper_H /
Piper_X / Piper_L, Dobot CR5 (`payloads/ROS/fiveages.usda`). New fiveages
robots must follow the same `ROS_Clock` pattern — never bind `/clock` to
`OnPlaybackTick`.

## 5. Env / Graph checklist

```
- [ ] targetPrim = ArtRoot prim (G1D: /World/G1D; W2: /World/FiveAges_W2/base_link)
- [ ] mode = full_body | split_body | demo
- [ ] robot = unitree_g1d / … OR empty + Robot=fiveages_w2|fiveages_w2r (name == package stem)
- [ ] enableArmsTargetManager; type empty → EE / EE_Left|Right / Left_Arm|Right_Arm EE → 运控 key (PascalCase→lower except CR5 AG2F*)
- [ ] Physics=physx pinned; ROS=fiveages; PhysX-only PhysicsScene; /clock if useSimTime
- [ ] `ROS=fiveages` clock node `resetOnStop=1` (sim time restarts on Stop)
- [ ] `/clock` comes from the on-demand `ROS_Clock` graph (`OnPhysicsStep` → `PublishClock`), **not** `ROS_Control/OnPlaybackTick` (see §4.1)
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
| TF | Per-robot `/<ns>/tf` (+ static). Do **not** merge fleets onto one `/tf`. Frame names may stay `base_link`. Isaac publishes `world → odom → base_footprint` only. `robot_state_publisher` (spawned with the latched xacro) publishes `base_footprint → base_link` and the arm/eef tree, including fixed frames such as `right_eef` |
| `world→odom` | **Play-first latch** of chassis world pose (`LatchWorld2Odom`). Spawn G1D at **z=0**. Not every-frame current base. **Must re-latch on Stop→Play** via simTime rewind / frame drop / (`odom≈0` + hub jumped >10cm); settle ~5 ticks. ScriptNode `setup()` does not re-run — without this, TF sticks after moving ArtRoot while Stopped. Reload ROS payload after editing |
| `odom→*` | `IsaacComputeOdometry` **relative** displacement from Play (`/odom` + `odom→base_footprint`) |
| WBC `tf_topic` | Must be relative `tf` (→ `/<ns>/tf`), **not** absolute `/tf`. Otherwise activate probe misses Isaac TF and disables unlocked-base planning |
| Chassis topics | Payload `topicName` = relative `cmd_vel` / `odom` (+ `nodeNamespace`) |
| ATM | Node `arms_target_manager_<ns>` under `/<ns>`; spawn also remaps `/tf:=tf` and `/tf_static:=tf_static` (TransformListener ignores `__ns` for absolute `/tf`) |
| RViz / tf2 CLI | Default listens `/tf`. Use `ros2 run tf2_ros tf2_echo world base_link --ros-args -r /tf:=/g1d/tf -r /tf_static:=/g1d/tf_static` (or RViz TF topic remaps). Empty `/tf` with ns enabled ≠ missing Isaac TF |
| Helper nodes | Same short name under ns prefix: `/<ns>/fasim_robot_description`, `/<ns>/fasim_ros2_control_launch`, `/<ns>/robot_state_publisher` (or `robot_state_publisher_<ns>`). RSP remaps `/tf:=tf` and `/tf_static:=tf_static` |

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
| home/moveJ duration too short (e.g. 5s→~1.5s wall); CM `period` jitters `0/0.015/0.020` | `/clock` bound to `OnPlaybackTick`. Move it to an on-demand `ROS_Clock` graph driven by `OnPhysicsStep` (§4.1) |
| `Physics OnSimulationStep node detected in a non on-demand Graph` | `OnPhysicsStep` placed in an `Automatic` graph. Use a dedicated on-demand graph with `pipelineStage="pipelineStageOnDemand"` (§4.1) |
| Continuous track offset on mobile WBC | `control_base_frame`+`marker` must be `world`; vr_follow stays footprint |
| tensors invalidated mid-run | Don’t flip EE/Physics variants on Play; fix snap first |
| G1D 进场景后再选 Physics → 双臂掉落 | mount/env sticky Physics 与 session 打架 → Assembler `disjointed`。G1D 已去 sticky；子件由父 Physics payload push；换引擎卡住 Reload |
| Newton 下 fiveages 异常 | `IsaacSimSystem` 支持 Newton；先确认 `Physics=mujoco` 与引擎一致、资产 mujoco 轨完整。勿用 env pin PhysX 绕过 |
| `get_ros2_control_robot_description returned empty` (G1D / caster xacro) | Post-guard partial AMENT + stub `/opt/ros/.../component_models` shadows ros2_ws. `_prepare_host_ros_env` must clear AMENT/COLCON/CMAKE/PYTHONPATH before `source setup.bash` so overlay wins |
| `dlopen … libtracetools.so: cannot open shared object file` / ocs2_wbc load fail | `./run.sh` must use `FASIM_FULL_HOST_ROS=1` (keep `/opt/ros` on LD) + exclude pink. Do **not** strip underlay LD. `ensure_isaac_ros2_library_path` keeps ws `lib/` before `/opt/ros` so apt `libocs2_*` cannot shadow |
| apt `libocs2_*.so` shadows workspace (pinocchio SONAME mismatch) | Overlay-first LD; or remove apt ocs2 packages. Workspace sourced ⇒ its `install/*/lib` must precede `/opt/ros/*/lib` |
| W2 `inputs:robot` empty + `Robot=none` | Set `Robot=fiveages_w2` or `fiveages_w2r`, or hardcode `inputs:robot` |

## 关节限速：URDF ↔ USD 对齐

- `physxJoint:maxJointVelocity` 的单位是 **deg/s**（PhysX 与 Newton 都这么读）：`68.754931` = 1.2 rad/s、`572.9578` = 10 rad/s、`359.81747` = 6.28 rad/s。
- in-process CM 合成 URDF 时，导出器缺 velocity 的关节**不再统一填 10 rad/s**，而是读
  `physxJoint:maxJointVelocity`（deg/s → rad/s）；prismatic 跳过（该属性在线性关节是 m/s）。
  日志：`sanitized non-finite joint limits: head_joint1.velocity=1.2, ...`。
- 关节可能挂在 ArtRoot 的**兄弟** scope（W2：`…/FiveAges_W2/joints`），所以取限速要遍历
  `_synth_joint_scope_path()` 返回的根，而不是 ArtRoot prim 本身。
- 想让运控侧和仿真一致，先确认 USD 里给到了 cap；没写就退回默认 10 rad/s。

## Related

- `USDA-OCS2-PhysX-Mujoco` — articulation / Physics variants / DiffDrive
- `fasim-dexhand-asset` — EE Assembler + PhysX hand baseline
- robot_common_launch: `prepare_arms_target_manager_parameters`, `parse_task_info`

## 8. NavigationLaunch (Nav2 on Play)

`fasim.ros2.nodes.NavigationLaunch` mirrors the control node for navigation:

| | |
|---|---|
| Graph | `Graph/ROS_Navigation/NavigationLaunch` in `payloads/ROS/fiveages.usda`, `enabled=0` by default |
| Command | `ros2 launch <launchPackage> <launchFile> map:=<abs> nav2_profile:=… scan_topic:=… odom_topic:=… use_sim_time:=… use_rviz:=… publish_map_world_tf:=…` |
| Map | `inputs:map` 是 **string** 路径（不是 USD asset）：绝对路径直用，相对路径按打开的 env 层（root layer）解析，所以 `env/` 里写 `../../maps/<name>/map.yaml` 就指到项目目录 |
| Lifecycle | `compute-on-request`; spawns a process group, SIGTERM on timeline STOP, re-launches on next Play |
| Logs | `inputs:logFile`, else `<map dir>/nav2.log` |

Rules that bite:

- **新增/修改 `.ogn` 后必须让 OmniGraph 重新生成节点接口**：生成结果缓存在
  `~/.cache/ov/ogn_generated/<graph 版本>/<ext名>-<ext版本>/`，**按扩展版本做 key**。
  只改 `.ogn` 不改 `config/extension.toml` 的 `version` 时，Kit 会用旧缓存，报
  `Could not find node type interface for 'fasim.ros2.nodes.NavigationLaunch'`。
  做法：改版本号（或删掉该缓存目录），然后重启 Isaac Sim。

- Pass **only** arguments the launch file declares (`navigation_isaac_gt.launch.py` rejects unknown `k:=v`). Extra args go through `inputs:extraArgs`, space separated.
- `map->world` is a fixed identity TF in that launch: the occupancy map must be drawn in USD world coordinates. Otherwise add your own offset TF and set `publish_map_world_tf=0`.
- Nav2 needs `/clock`; `ROS=fiveages` already publishes it (`PublishClock` in `ROS_Control`). Keep `use_sim_time=1`.
- `ROS=fiveages` also pushes chassis `ROS=enable`, which owns `cmd_vel` / `/odom` / `world→odom→base_footprint`; do not start a second odom source.
- `robot` empty → node reads the `Robot` variant from `targetPrim` ancestors (`robot` legacy alias → `fiveages_w2`).

