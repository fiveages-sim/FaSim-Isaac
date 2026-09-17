---
name: fasim-dexhand-asset
description: >-
  Build FaSim-Isaac unified dexterous-hand USD assets (LinkerHand o6/o7,
  BrainCo Revo1/Revo2, RobotEra Xhand1, etc.) from separate left/right USDA
  imports: confirm mirror axis (X like o6 vs Y like o7/Xhand1) before
  conversion; PhysX-safe mirror bake (root scale always (1,1,1)); Hand/left|right
  morphology; Side variants with left_hand_/right_hand_ rename; Physics variants;
  validated sim baseline (maxJointVelocity=500000 frozen; G1 hold D=0.06 K=0.1
  F=30 / thumb D=0.08 K=0.12 F=50 + jointFriction=0.5; μs=2.0 contactOffset=0.002;
  distal CCD; self-collision + FilteredPairs). LED glow uses Head_V1 glTF
  emission (emissive_factor × strength; inverted LED normals look like
  white metal with no glow). Warn AssemblerFixedJoint must match mount TF;
  never ship build scripts or raw *_left/*_right trees. Does NOT write
  arm EE mounts.
---

# FaSim Dexterous Hand Asset

从**左右手各自**的 ROS2 导入 + Asset Transformer USDA，做成与
`robots/dexhands/LinkerHands/o7/` / `o6/` 同构的**单一入口**资产。

## 交付状态

| 型号 | 结构 Side L/R | 关节基线 | 自碰+Filter | 碰撞摩擦 | 备注 |
|------|---------------|----------|-------------|---------|------|
| **o6** | ✅ | **G1 hold 已同步 o7** | ✅ | ✅ μ2.0+contactOffset | W2:`dexhand_o6_new`；`Flange=default` = AG2F90 金属 + 黑打印件 |
| **o7** | ✅ | **G1 完全通过** | ✅ 保留捏合 | ✅ | W2:`dexhand_o7_new` 抓瓶不穿不粘可抬 |
| **Xhand1** | ✅ Y 镜像 | 对齐 §0.1；12 DOF 无 mimic | ✅ | ✅ | `RobotEra/Xhand1`；右手掌单独烘焙；§4.1 |
| **L6** | ✅ Y 镜像 | 对齐 §0.1；mimic 0/0 保留 gearing | ✅ | ✅ | `LinkerHands/L6`；四指不翻 mesh；M6 EE=`LinkerHand_L6`；Look；Flange |

**黄金参考：**

- **结构 / Side / 挂载**：`LinkerHands/o6/`、夹爪 `grippers/Jodell/RG75/`
- **官方自碰写法**：`dexhands/Inspire/module_5_end-checkpoint_3/`（开自碰 + `PhysicsFilteredPairsAPI`）
- **关节 PhysX（o6/o7 已跑通）**：§0.1（含 K/D/F 调参说明）
- **指示灯发光**：`FiveAges/Gen2/components/Head_V1/payloads/materials.usda` 的 `Material "light"`（§8.1）

**资产目录只保留交付物**（入口 USDA + `payloads/` + `Textures/`）。  
禁止放入：`o7_left/`、`o7_right/`、`_*.py` 构建脚本、`__pycache__/`、`transform_report.json`、已烘焙的源 mesh（如 `hand_link.usd` / `.glb`）、小写重复树（如 `xhand1/`）。算法写在 [reference.md](reference.md)。

**待处理批次：**

| 源（左右分文件夹） | 目标统一文件夹 |
|--------------------|----------------|
| `LinkerHand_o6_*` | `LinkerHands/o6`（✅） |
| `LinkerHand_o7_*` | `LinkerHands/o7`（✅） |
| `BrainCo_Revo1_*` | `BrainCo/Revo1`（LED glTF 已按 Head_V1 跑通；灯面法线已翻朝外） |
| `BrainCo_Revo2_*` | `BrainCo/Revo2`（LED `green_light` 同套；右手四指近端 ×Rz(180°)） |
| `RobotEra/xhand1`（小写导入） | `RobotEra/Xhand1`（✅ Y 镜像；右手掌单独烘焙；§4.1） |
| `LinkerHands/L6`（单手导入） | `LinkerHands/L6`（✅ prim `L6`；Y 镜像；四指不翻 mesh；M6 EE=`LinkerHand_L6`；Look；Flange） |

---

## 0. 问诊（先问再改）

```
【DexHand 问诊】
1. 左右源路径？（*_left / *_right 入口 usda）
2. 目标品牌/型号文件夹名？入口 prim 名？（例 LinkerHand_o7）
3. 关节前缀？默认 left_hand_ / right_hand_
4. default Side 用哪只手形态？通常 = 左手
5. 【必确认】右手镜像轴 / 做法？未确认前禁止套用另一轴
   - [ ] X 镜像（o6）：link 位姿 X 翻 + 视觉 Sx=-1 + 碰撞 *_xflip；根 (1,1,1)
   - [ ] Y 镜像（o7 / Xhand1）：PhysX 安全烘焙（§2B / reference §Y / §K2）；根 (1,1,1)；Side/right 关节 Y 镜像 + X/Z 轴 ×Ry(180°)
   - [ ] 其它
   禁止：仅靠根负 scale 交付（Play 挂臂会跳）。
6. 驱动 / mimic：对齐 §0.1（先 maxJointVelocity=500000，再抄 o6/o7 D/K/F）
7. 自碰：开 + FilteredPairs（仿 Inspire；静位用 Self-Collision Detector 复核）
8. 碰撞摩擦：PhysicsMaterialAPI 写在 collider 上（勿单独 Material 绝对路径）
9. 做完删除源左右文件夹？[ ] 是 [ ] 否
```

**挂载（本 skill 不写臂 EE 细节以外的运控图，联调必查）：**

1. `AssemblerFixedJoint` 的 `localPos0/localRot0` **=** 挂载 `translate/orient`（否则 Play 跳变）。
2. 右手根 / 挂载 **禁止** 负 scale；形态必须已烘焙到根 `(1,1,1)`。
3. **挂载灵巧手自碰（已固化 · o6/o7 × M6/W2）**  
   - 单独手：手内 `enabledSelfCollisions=1` + FilteredPairs  
   - 挂载后手 `root_joint` inactive，手指并进父 articulation  
   - **自碰开关写在 Physics/physx**（EE sublayer 盖不掉）：`Physics` 在 `variantSets` 更靠前  

   | 层 | 文件 | 作用 |
   |----|------|------|
   | 臂物理 | `M6_CCS/payloads/Physics/physx.usda` | `root_joint` self-col=**1** + 臂杆 `NonDexHand` |
   | 机身物理 | `W2/payloads/Physics/physx.usda` | `base_link` self-col=**1**；subLayer 静音层 |
   | 机身静音 | `W2/payloads/Physics/body_self_collision_mute.usda` | 躯干+底盘+左右臂 `NonDexHand` + dexhand excludes（**与臂选型无关**） |
   | 灵巧手 EE | `.../EE/dexhand_self_collision.usda` | 通用规范：给臂 `NonDexHand` 加手 excludes（不绑厂商名） |

   **新 dexhand EE 接入清单**  
   1. 挂载：`root_joint` inactive + FixedJoint（与父同树）  
   2. EE usda `subLayers` → `@./dexhand_self_collision.usda@`，并把 `/…/tcp/<HandPrim>` 写入该文件 excludes  
   3. 同步左右腕路径到 `body_self_collision_mute.usda` excludes  
   4. 夹爪不要写入 excludes（留在 NonDexHand 自滤即可）  

   - **模块化**：静音在 W2 `Physics`，任意 `Arm_Left`/`Arm_Right` 组合（含单侧 M6、混装 ART7）都有机体静音，不绑死左臂挂载文件。  
   - **坑**：`includes` `link7` 会展开到 EE；不 `excludes` → 手指穿模。手保持**未分组**。  
   - **其它 EE / ROS2**：只动碰撞过滤；不改 Graph、关节名、ArticulationRoot、`robotJoints/Links`。  
   - **禁止**在 env 里为自碰开洞作为长期方案。  



---

## 0.1 接触物理基线（o6/o7 已跑通 · 2026-09）

灵巧手仿真主路径已通：`maxJointVelocity` 消炸 → 自碰 FilteredPairs → 摩擦 → drive 手感后调；挂臂用「父树自碰 + NonDexHand 静音 + EE excludes」。

| 项 | 值 / 要求 | 说明 |
|----|-----------|------|
| `physxJoint:maxJointVelocity` | **500000** | **P0 冻结**；与「被顶开/肌无力」无关，勿靠降它治软 |
| 主动 drive（o7 G1 hold） | 四指 **D=0.06 K=0.1 F=30**；拇指 **D=0.08 K=0.12 F=50** | 抗反向顶开；**K 仍 &lt;0.2**（≥0.2 易桌面吸附）；D 同比抬高防颤 |
| 关节被动 | `jointFriction≈0.5` `armature≈0.001` | 近似非回驱/锁定感；比猛加 K 更安全 |
| 接触冲量 | `maxDepenetrationVelocity=2.0`；solver vel≥4；指尖 CCD | 穿模退出靠 depen↑；过小易卡在穿透里 |
| Mimic | `dampingRatio=0` `naturalFrequency=0` `offset=0` | 硬耦合；勿随意加 compliance |
| 关节限位 | flex **lowerLimit≥0** | 负 lower=允许手背超伸 |
| 自碰 | **On** + `PhysicsFilteredPairsAPI` | 滤假接触，**保留**拇指 tip↔指尖捏合 |
| 摩擦 | collider μs=**2.0** μd=**1.6** rest=0 + `frictionCombineMode=max` + `improvePatchFriction` | 场景物体也须同套；勿写 `material:binding:physics = None` |
| 穿模抑制 | `contactOffset=0.002`；`maxDepenetrationVelocity=2.0`；指尖 `enableCCD` | F 大时靠接触厚度/CCD，勿靠猛降 F |
| 碰撞 | mesh + `convexHull`；det&gt;0 | |
| Solver | pos=64 · vel=4 | |

```usda
# 四指（G1 hold · o7 完全通过 · o6 已同步）
float drive:angular:physics:damping = 0.06
float drive:angular:physics:maxForce = 30
float drive:angular:physics:stiffness = 0.1
# 拇指
# damping=0.08  stiffness=0.12  maxForce=50
uniform token drive:angular:physics:type = "force"
float physxJoint:jointFriction = 0.5
float physxJoint:armature = 0.001
float physxJoint:maxJointVelocity = 500000
# collider
float physics:staticFriction = 2.0
float physics:dynamicFriction = 1.6
float physxCollision:contactOffset = 0.002
float physxRigidBody:maxDepenetrationVelocity = 2.0
# distal: bool physxRigidBody:enableCCD = 1
```

### Drive 参数怎么调（force 型）

力矩近似：`τ ≈ K·(q_target − q) − D·q̇`，再被 `maxForce` 截断。

| 参数 | 作用 | 调大 | 调小 |
|------|------|------|------|
| **stiffness (K)** | 小偏差下的回持刚度 | 难被顶开，但易吸附 | 软、易被推开 |
| **damping (D)** | 耗散振荡 | 加 K 时必须同比抬，防吸附颤 | 易颤 |
| **maxForce (F)** | 力矩上限（大误差时力气） | 捏得住；过大易穿模 | 饱和后仍没劲 |
| **jointFriction / armature** | 被动抗回驱（仿真「锁定」） | 更难被反向掰开 | 更飘 |
| **maxJointVelocity** | 关节角速度上限 | 只限闭合/甩动速度 | **不**增加持力 |
| **contactOffset** | 提前生成接触 | 减轻穿模 | 接触发虚 |

**进度（2026-09 · o7 G1 DONE · o6 synced）：**

- ✅ **o7 完全通过**（W2 `dexhand_o7_new`）：movej 合拢抓瓶可抬、触桌不崩不粘、大力不穿模、慢滑已消；drive/摩擦/接触基线冻结  
- ✅ **o6 已同步同套 G1**（资产 + `o6/env/test.usda` + W2 `dexhand_o6_new.usda`）  
- 冻结值：四指 **K=0.1 D=0.06 F=30**；拇指 **K=0.12 D=0.08 F=50**；`jointFriction=0.5`；μ **2.0/1.6** + `max`；`contactOffset=0.002`；depen **2.0**；指尖 CCD；`maxJointVelocity=500000`  
- 场景纪律：物体须自带 `PhysicsMaterialAPI`+`PhysxMaterialAPI`；**禁止** `material:binding:physics = None`；桌建议 kinematic  

**抓取 / 抗顶开策略（o6/o7 G1 · 勿用机械臂大 K）：**

- 真机蜗轮/棘轮「只能正向」PhysX 无原生单向锁；用 **中等 K + 高 F + 高 jointFriction** 近似  
- 小角度被顶开：主要靠 **K 与 jointFriction**（F 要等误差攒大才满额）  
- 桌面吸附 → **降 K** 或再抬 **D**，不要冲到 K≥0.2  
- 穿模卡住 → 先加 **contactOffset/CCD/depen**，再考虑略降 F（保持 K）  
- **maxJointVelocity=500000** 冻结  
- 曾炸飞：`K≈1.5`；曾吸附：`K≈0.2` → 四指避开该区  


**自碰 FilteredPairs：** 相邻连杆默认不碰；只滤静位假重叠。拇指 tip↔其他指 tip **不要滤**（捏合要顶住）。工具：`Tools → Robotics → Asset Editors → Robot Self-Collision Detector`。

---

## 1. 总流程

```
- [ ] A. 审计左右源：prim、关节、碰撞负 scale、mimic、Physics
- [ ] A2. 确认镜像轴（问诊 §5）
- [ ] B. 按轴烘焙碰撞 / 形态；根 scale=(1,1,1)；碰撞 det>0
- [ ] C. 建 Hand/left|right + 共享 Physics/Robot/Sensor
- [ ] D. Side default|left|right（改名；right 切 Hand/right；Y 轴时关节 Y 镜像 + X/Z 纠转向）
- [ ] E. 入口 USDA；§0.1 关节基线 + 自碰/Filter + 摩擦
- [ ] F. 校验：拇指解剖侧、关节 active、碰撞 det、Play 挂载不跳、握拳不炸
- [ ] G. 删除源左右树与构建脚本；资产目录干净
- [ ] H. 臂 EE 另任务（FixedJoint=挂载 TF）
```

---

## 2. 碰撞负 scale 与镜像

### 现象

镜像引入负 scale → PhysX 无法烹饪 det&lt;0 的 mesh collider → 丢碰撞 + Physics Tasks 闪烁。  
根负 scale → 编辑态好看，**Play 挂臂跳变**。

### 方案表

| 方案 | 型号 | 形态 | 碰撞 | 根 | 臂挂载 |
|------|------|------|------|----|--------|
| X | o6 | link X 翻 + 视觉 Sx=-1 | `*_xflip`，局部 (1,1,1) | (1,1,1) | (1,1,1) + FixedJoint=TF |
| Y | o7 | link `T'=(Sy R Sy, Sy t)` + 视觉局部 Sy=-1 | `*_yflip`，局部 (1,1,1) | (1,1,1) | 同上 |

Y 细节与脚本级步骤 → [reference.md](reference.md)「PhysX-safe Y-mirror」。

### 验收

- 拇指解剖侧正确；碰撞 world det&gt;0  
- Play：无持续 Physics Tasks 闪烁  
- 挂臂 Play：位姿不跳；轻压桌/抓方块接近参考手  

---

## 3. 目标目录（交付）

```
{Brand}/{name}/
  {Name}.usda
  Textures/                 # 可选
  payloads/
    base.usda               # → Hand/left
    Hand/
      materials.usda        # 左右共用（Xhand1）
      robot.usda            # 无前缀关节（default Side）
      left/{base,instances,geometries}
      right/{base,instances,geometries}   # 只留右手独有 mesh
    Side/default|left|right.usda
    Physics/physics|physx|mujoco|none.usda
    Robot/  Sensor/
```

左右 `materials`/`robot` 相同则**提升到 `Hand/`**，不要各拷一份。右手 `instances` 可引用左手，只 over 掌/拇指独有几何。四指 mesh 与左手逐点相同时，右手 geometries **不要复制**，instance 指 `@../left/geometries.usd@`。

---

## 4. Side variant

| Side | 形态 | 关节 |
|------|------|------|
| default | 左 | 无前缀 |
| left | 左 | `left_hand_*` |
| right | 右 | `right_hand_*`；入口 `delete` 左 base → `Hand/right` |

Y 镜像 right：关节/质量 localPos/Rot 同步 Y 镜像；`axis=X|Z` 再 ×Ry(180°)（拇指 joint2/3）。

**`robotJoints`：** 共享 `Hand/robot.usda` 列的是无前缀名。`Side/left|right` payload 里 `rel=` / `delete rel` **盖不掉** base subLayer 的 `prepend rel`。必须在**入口** `variantSet "Side"` 的 left/right **花括号内**写显式 `rel isaac:physics:robotJoints = [ left_hand_* | right_hand_* ]`。

**切 Side 丢 Physics：** 根上不要 authored `Physics=physx`（挡 VariantSwitcher）。Isaac 改 Side 会按入口 `variants` 字典重写，session 里的 physx 被清掉，Switcher 写过一次就不再补 → Play 出现 `CreateJoint - no bodies` + jointFriction 警告。做法：每个 `Side/{default,left,right}.usda` 的根 prim **弱**写 `variants = { string Physics = "physx" }`（payload 层）；session `mujoco` 仍能盖过。

**`physx.usda`：** 必须同时 over 无前缀 **和** `left_hand_*` / `right_hand_*`。只写无前缀时，Side=right 的活动关节吃不到 PhysxJointAPI。

**Side/right 四指刚体：** 不要只 over COM。Physics 变体空着时 COM-only 没有 `RigidBodyAPI` → `middle/pinky/ring_joint2` 报 `CreateJoint - no bodies`。四指 overs 必须与拇指一样带齐 RB + Mass + FilteredPairs（COM 用 Y 镜像值）。

### 4.1 Xhand1 要点（2026-09 · Y 镜像）

入口：`robots/dexhands/RobotEra/Xhand1/Xhand1.usda`，prim `Xhand1`。12 独立 DOF，无 mimic。

| 项 | 做法 |
|----|------|
| 镜像轴 | **Y**（食指 −Y、小指 +Y）。先确认轴，禁止套 X。 |
| 根 / 碰撞 | 根 `(1,1,1)`；碰撞 det>0。有现成右手掌 mesh 则视觉/碰撞**共用**该 mesh，不要再 `Sy=-1` 或留一份 `*_yflip`。 |
| 四指 | **位姿** Y 镜像（`t'=(x,-y,z)`，`R'=Sy R Sy`）；**mesh 不翻**，引用左手 `mesh_2`–`mesh_7`。只翻掌+拇指、四指留在左手 Y 上 → 拇指会跑到小指侧。 |
| 拇指 / 手背 | 无现成右手 mesh 时：视觉可 `Sy=-1`，碰撞烘焙 Y-flip 后局部 `(1,1,1)`。Xhand1 已把 yflip 折回规范名，视觉碰撞共用。 |
| 右手掌源 | 用户给的 `hand_link.usd` 常是 **Y-up** + 根 `Rx(90°)`。用 World `ComputeLocalToWorldTransform` 把点/法线烤进 Z-up `/Geometries/mesh`，保留 `black`/`pad` GeomSubset。烤完**删除**源 `hand_link.usd` / `.glb`。 |
| 几何去重 | 右手 `geometries.usd` 只留独有：`mesh`（掌）、`mesh_1`（手背）、`mesh_8`–`12`（拇指）。禁止 `mesh` 与 `mesh_yflip` 各存一份相同点。 |
| instances | 右手薄文件：`references = @../left/instances.usda@`，只 over 掌/手背/拇指 geom。材质 `@../materials.usda@`。 |
| 材质 | 共用 `Hand/materials.usda`。Xhand1 `black`：`metallic_factor=0.8`（白/垫/灯不动）。MDL 相对 `Hand/` 为 `@../../Textures/pbr.mdl@`。 |
| `delete references` | Side=right 的 `delete` 左 base **经常删不干净**（与 Wuji 相同）。靠右手更强意见盖住；验收时看 composed npts / Y 范围，不要只看文件。 |

### 4.2 L6 要点（2026-09 · Y 镜像 + Look + Flange）

入口：`robots/dexhands/LinkerHands/L6/L6.usda`，prim `L6`。四指独立 DIP mimic（gearing 保留，`dampingRatio=0` `naturalFrequency=0`）。

o6 转接板已从 Hand 烘焙 mesh 拆成 **`Flange` 变体**（仿 AG2F90）：`payloads/Flange/default.usda` 在已有 `flange` link 上挂 AG2F120S 金属法兰 + `payloads/Flange/flange.usd` 黑打印件。默认 `Flange=default`。M6 o6 焊点仍是 `T z=0.0045, Rz=+90`。

| 项 | 做法 |
|----|------|
| 镜像轴 | **Y**（与 Xhand1 同）。根 `(1,1,1)`。 |
| 四指 | **位姿** Y 镜像；**mesh 不翻**，右手引用左手四指 geom。掌 + 拇指单独烤 Y-flip。 |
| 质量 | 整手 **607 g**；按 o6 连杆比分摊（勿均匀 9 g / 勿抄 0.989 kg/link）。 |
| Flange | 默认 `none`。`default` 复用 o6 的金属/打印件 usd；因掌在 flange 原点，整栈相对 o6 再沿 Z 移 **-0.024383**。 |
| 挂载 | `flange` 为 RB/`root_joint` 父；M6 EE 选项 **`LinkerHand_L6`**，prim 仍 `L6`。焊点 **`T z=0.028883, Rz=+180`**（0.0045+0.024383；不要抄 o6 的 Rz+90）；EE 选 `Flange=default`。FixedJoint 与 xform 同 TF。 |
| Look | `default` = 原白 `shell`/`finger`；`dark_gray` = 缎面电镀银 `(0.58,0.59,0.62)` metallic=0.88 roughness=0.32。橡胶/LED/`black_metal` 不动。 |

**Look 如何穿过 instanceable 材质：** `VisualMaterials/shell|finger` 在 instance 原型里 `references` `materials.usda`。`specializes` **盖不过** 这份 reference（合成栈里白色仍更强）。正确做法：`payloads/Look/dark_gray.usda` 把视觉 instance xform **和** 嵌套 Material 都设 `instanceable=false`，再本地写 `base_color_factor` / `metallic_factor` / `roughness_factor`（local > reference）。`default` payload 为空，保持原白。切 Look 后必须 **Reload Stage**。右手 `instances` 引用左手，同一套 overs 生效。

---

## 5–6. Physics / 自碰 / 摩擦

见 §0.1。参考实现：`o6` / `o7` 的 `Physics/physics.usda` + `physx.usda`，以及 Inspire `module_5_end-checkpoint_3`。

---

## 8. 校验清单

```
Side L/R/default 形态与关节前缀正确
碰撞 mesh convexHull；det>0；collider 有摩擦
mimic 0/0；drive + maxJointVelocity=500000 对齐 §0.1
enabledSelfCollisions=1；静位假接触已 FilteredPairs
资产目录无 *_left/*_right、无 _*.py、无 __pycache__、无已烘焙源 mesh（hand_link.usd 等）
切 Side=right 后 Physics 仍为 physx（或 session mujoco）；Play 无 CreateJoint no bodies
挂臂：FixedJoint = 挂载 TF；Play 不跳
接触：握拳/指垫互碰不飞；可抓桌上物体（物体侧也要摩擦）
LED：GeomSubset 有面；emissive_factor≠0；灯面法线朝外；Reload Stage 后发光
L6 Look：default 仍白；dark_gray 只改 shell/finger，切变体后 Reload
L6 Flange：单独打开为 none；M6 `EE=LinkerHand_L6` 为 default（AG2F90 银法兰 + 黑打印件），掌在板外侧、Rz=+180；Play 不跳
```

---

## 8.1 LED 发光（glTF · Head_V1 / Revo 已跑通 · 2026-09）

Isaac RTX 认的是本地 `Textures/pbr.mdl` 的 **gltf_material**，不是 OmniPBR。

```
emission = emissive_factor * max(0, emissive_strength)
emissive_factor 默认 (0,0,0)  →  只写 strength 完全不亮
```

**黄金抄本：** `robots/humanoid/FiveAges/Gen2/components/Head_V1/payloads/materials.usda` → `Material "light"`。  
仓内对齐：`BrainCo/Revo2` `green_light`、`BrainCo/Revo1` `light`、`Astribot/S1` `light`（青色因子）。

```usda
asset info:mdl:sourceAsset = @../../Textures/pbr.mdl@
token info:mdl:sourceAsset:subIdentifier = "gltf_material"
int inputs:alpha_mode = 2                    # blend
color3f inputs:base_color_factor = (1, 1, 1) # 白底，颜色走 emission
color3f inputs:emissive_factor = (0, 1, 0.002)  # LED 色；Revo 绿 / Head 青 (0, 0.968, 1)
float inputs:emissive_strength = 4950
float inputs:metallic_factor = 1
float inputs:roughness_factor = 0
float inputs:ior = 1.9
float inputs:transmission_factor = 0.23622048
```

`pbr.mdl` 相对 `payloads/Hand/materials.usda`（或机身 `payloads/materials.usda`）解析。改完必须 **Reload Stage**（Stop/Play 不够）。

| 现象 | 原因 | 做法 |
|------|------|------|
| 仍是原色塑料、不发光 | 只有 `emissive_strength`，因子仍是 0 | 写非零 `emissive_factor` |
| 亮面白色金属、不发光 | 白底+金属 1 已绑上；**灯面法线朝内**，RTX 只正面发光 | 翻 GeomSubset 绕序 **且** 翻转 faceVarying normals（反转 corner 顺序并取反） |
| 改 OmniPBR `enable_emission` 仍不亮 | 本仓 LED 不走 OmniPBR | 改回 glTF，抄 Head_V1 |

法线朝向：灯面法线 · (面心 − mesh bbox 中心) **应 > 0**（朝外）。Revo2 `green_light` 100% 朝外；Revo1 `light` 曾 100% 朝内，翻面后发光。右手视觉 `Sx=-1` 会再翻世界法线，若仅右手不亮再单独处理。

GeomSubset：`familyName=materialBind`，`indices` 非空；绑定 `VisualMaterials/<mat>`，不要另起 OmniPBR 材质名。

**禁止：** 把 OmniPBR 当 FaSim 指示灯默认方案；只改 strength 不写 factor；只翻 winding 不改 faceVarying normals（Hydra 仍用旧朝内法线）。

---

## 9. 明确不做

- 在本 skill 里写满机械臂 EE（只给挂载约束）  
- 把胶囊/方盒简化碰撞当最终交付  
- 把构建脚本留在 `robots/dexhands/...` 资产树内  
- 再为 D/K/`maxForce` 做长时间穷举（基线已冻结）  

## 更多

[reference.md](reference.md) — Y 烘焙、Isaac 片段、Inspire 对照、差距表。

---

## N. Newton + MuJoCo（MJWarp）灵巧手调试经验（o7 实战追加）

> **本节为追加内容**，不改写上文 PhysX / 结构条款。PhysX 轨仍以 §0.1 为准。  
> 仓内实现：`LinkerHands/o7/payloads/Physics/mujoco.usda` + `env/newton_smoke.usda` + `env/test_finger_targets.py` + `env/newton_jog_dofs.py` + `env/newton_thumb_debug.py` + `env/newton_kill_callbacks.py`。  
> 双轨写法总览另见 skill `isaac-mujoco-physics-variant`。

### N.1 架构要点

| 项 | 要点 |
|----|------|
| 前端/后端 | Isaac **Newton** 引擎 + USD `Physics=mujoco` 轨（`MjcJointAPI` / `MjcActuator` / `NewtonMimicAPI`） |
| 隔离 | **只改 `mujoco.usda` + Side 内 Mjc 字段**；勿改已验证的 `physx.usda` / 摩擦 / FilteredPairs |
| 共用 | `physics.usda` 质量·关节·**FilteredPairs**；Newton 会读 UsdPhysics 过滤对 |
| 自碰开关 | PhysX：`physxArticulation:enabledSelfCollisions`；Newton：`newton:selfCollisionEnabled`（写在 mujoco 轨） |
| VariantSwitcher | 根上**不要** authored `string Physics=physx` 才能首次自动切；写入 session 后换引擎需 **Reload Stage**，否则卡住 |
| 场景 | Newton 测需 `MjcSceneAPI`；烘焙 Hydra RenderProduct + 远程 HTTPS 道具易导致 **Play 黑屏** → 用精简 `newton_smoke.usda` |

### N.2 驱动原理（面板上没有 ctrl）

`MjcActuator` Property 只有增益/限幅配置，**没有**运行时命令滑条。

```
gainType=fixed, biasType=affine
gainPrm=[kp], biasPrm=[0, -kp, -kd]
F = kp*(ctrl - q) - kd*qdot
```

运行时 `ctrl` / 关节目标通过 **Script Editor + `Articulation.set_dof_position_targets`** 注入（官方 Newton 用法），单位是 **弧度**。

### N.3 血泪坑（必读）

1. **单位**：`set_dof_position_targets(45)` = **45 rad**，会把手指打穿/折叠。正确：`40 * pi/180`。改 40→60 若仍炸，多半仍是弧度量级问题。  
2. **mimic 符号**：PhysX `gearing<0` 搬到 `newton:mimicCoef1` 时常对打。o7 验证：**`coef1 = +abs(PhysX gearing)`** 四指丝滑。方向反了再整体翻号。  
3. **拇指 DOF 多 / 被顶到上限**：o7 拇指 3 驱动 + 2 mimic。典型症状：`thumb_joint3` 在 ~1s 内爬回 **~40–60°**（近上限），teleport 到 0 也拉不回 → **自碰假接触**，不是重力/PD。见 **N.7**（勿先改 `physics.usda` FilteredPairs）。  
4. **无 Physics Inspector**：Newton 下用脚本点动（见 N.5），不要指望 PhysX Inspector。  
5. **爆炸后必须 Reload Stage** 再测；Stop 不够。  
6. **`time.sleep` 不会推进 Play**：poll 会读到同一帧；应肉眼等 1–2s 再跑 `snapshot`，或用 `set_dof_positions` 瞬移对照。  
7. **禁止粘性 timeline 订阅**：`hold_zero_frames` + 错误的 `remove_subscription` 会留下僵尸回调；Reload 后仍刷 `Instance is not valid`。只用单次脚本；紧急清 `newton_kill_callbacks.py`，不行就重启 Kit。  
8. **Mute 必须打到真实 collision mesh**：LinkerHand 碰撞在 **instanceable 原型内部的 `mesh_N`** 上（如 `.../collisions/thumb_*/mesh_5`）。只在 instance 根 Xform 写 `collisionEnabled=0` → **USD 看起来改了、Newton 仍碰撞**。正确：`instanceable=false` + 对嵌套 `mesh_*` 设 `physics:collisionEnabled=0`（见 N.7）。

### N.4 调参起点（o7 mujoco，可再调）

| 关节 | kp / kd（actuator） | armature / damping（MjcJoint） | 备注 |
|------|---------------------|--------------------------------|------|
| 四指驱动 | 5 / 1.5 | 0.002 / 0.02 | 40° curl 已验证稳定 |
| 拇指驱动 | 2 / 2.5（抑抖方向） | 0.005 / 0.08 | 高频小抖时先动这两项 |
| mimic 从动 | 无 actuator | 同侧被动参数 | 只改 `NewtonMimicAPI` |
| ctrlRange | **弧度**（约 0～1.36 对应 0～78°） | | 与 Articulation API 一致 |

### N.5 脚本交互（替代 Inspector）

路径均在 `robots/dexhands/LinkerHands/o7/env/`：

| 脚本 | 用途 |
|------|------|
| `newton_smoke.usda` | 无黑屏冒烟场景 |
| `test_finger_targets.py` | 四指 40° 开合回归 |
| `newton_kill_callbacks.py` | **紧急**：清掉上次 timeline 订阅僵尸；不行就重启 Isaac Sim |
| `newton_thumb_debug.py` | 拇指：`dump_cols` / teleport / sample / selfcol / 真实 mesh mute（**禁止**粘性 timeline） |

流程：Play → Script Editor 跑 jog / thumb 脚本。Thumb **不要**再用 hold_zero_frames 类订阅。  
若 Play 刷屏 `Instance is not valid`：先跑 `newton_kill_callbacks.py`，仍刷则 **退出 Isaac Sim 重开**（Reload 清不掉旧闭包）。

### N.6 验收阶梯

1. P0：`newton_smoke` Play 无报错、不黑屏；PhysX 回归手感不变  
2. P1：脚本四指稳定弯曲 + mimic 跟随  
3. P2：拇指静止无抽搐、零位不爬升；`selfCollisionEnabled=1`  
4. P3：握拳/邻指自碰 + 拇指 tip 捏合；再上 W2 运控  

### N.7 自碰：PhysX FilteredPairs vs Newton 拇指修复（o7 已验证）

#### 原则

| | PhysX | Newton（o7 结论） |
|--|-------|-------------------|
| 开关 | `enabledSelfCollisions=1` | `newton:selfCollisionEnabled=1`（保持开） |
| 假接触处理 | `physics.usda` **FilteredPairs**（相邻连杆 PhysX 常自动滤） | **优先**在 `mujoco.usda` mute 肇事 **collision mesh**；**不要**为 Newton 大改共享 FilteredPairs |
| 捏合 | **保留**拇指 tip ↔ 其他指 tip | 同：保留 `thumb_distal`（`mesh_9`）碰撞 |
| 隔离 | — | PhysX 轨不加载 `mujoco.usda` → mute **不影响** PhysX |

#### PhysX 现有 FilteredPairs（`physics.usda`，**未为 Newton 改动**）

掌/指尖侧（节选，与 §0.1 一致）：

- `flange` 过滤：各指 `*_distal`/`*_middle` + `thumb_distal` / `thumb_metacarpals` / `thumb_metacarpals_base2` / `thumb_proximal`（**不含** `thumb_metacarpals_base1`：与 flange 经 `thumb_joint3` 相邻，PhysX 自动不碰）
- 拇指链：`base1`/`base2`/`metacarpals` 过滤邻指近端 + 拇指远端等；**`thumb_distal` 自身无 FilteredPairs** → tip↔指 tip 可碰
- 四指邻指交叉过滤照旧

→ PhysX 靠「自碰 On + FilteredPairs + 相邻自动滤」已正常；**不要为 Newton 去改这张表**。

#### Newton 拇指故障与修复

**现象**：自碰 On 时 `thumb_joint3` ~1s 爬到 ~50° 并抽搐；`selfcol_off` 立刻正常。

**错误路径**：

1. 在 `physics.usda` 大面积加 FilteredPairs → 动到 PhysX，且未必修 Newton  
2. 只在 instance 根设 `collisionEnabled=0` → `dump_cols` 显示改了，**geom 仍在**  

**正确路径（已固化在 `mujoco.usda`）**：

```
thumb_metacarpals_base1/.../mesh_5  collisionEnabled=0
thumb_metacarpals_base2/.../mesh_6  collisionEnabled=0
thumb_metacarpals/.../mesh_7        collisionEnabled=0
thumb_proximal/.../mesh_8           collisionEnabled=0
thumb_distal/.../mesh_9             保持开（捏合）
+ 各层 instanceable=false（才能写入嵌套 mesh）
selfCollisionEnabled=1
```

语义：Newton 下去掉拇指 **非 tip** 自碰（拇指内部 + 易与掌假重叠的 CMC/proximal）；**保留 tip 与四指 tip**；全局自碰仍开（四指互碰等）。

#### 诊断清单（再遇到同类问题）

1. A/B：`selfcol_off` 好 → 自碰问题；`selfcol_on` 坏 → 继续  
2. `dump_cols`：确认 `.../collisions/.../mesh_N` 的 `collisionEnabled`，**不要**被 joints 上的 schema 默认项迷惑  
3. 运行时 mute 必须 `SetInstanceable(False)` 再写嵌套 mesh（`newton_thumb_debug.py`）  
4. 资产层 mute 只写 **`mujoco.usda`**；改完 **Reload Stage**  
5. 验证：`selfcol_on` + Play 零位稳定 + `teleport_zero` 后不再爬升  

#### 脚本 MODE 速查（`newton_thumb_debug.py`）

`dump_cols` → `selfcol_on` → `teleport_zero` → 等 2s → `sample_now`  
可选：`thumb_col_off`（非 tip）/ `thumb_all_col_off` / `palm_col_off`  

### N.8 整机挂载自碰（Newton，对齐 PhysX 意图 / 不同实现）

> 挂载后手 `root_joint` inactive，并入父 articulation。父轨必须自碰 On，并静音机体/臂，同时留出手指出自碰。  
> PhysX 表见上文「挂载灵巧手自碰」；**Newton 不能照搬 CollisionGroup `filteredGroups`**。

#### 为何整机穿模 / 为何「和自己较劲」

| 现象 | 原因 |
|------|------|
| 手指穿模 | 父 `newton:selfCollisionEnabled=0`（整树自碰关） |
| 开自碰后整机卡死摔倒 | 自碰 On，但 **NonDexHand 未真正进入 Newton 过滤** |

**两处资产坑（已修）：**

1. **根 authored `Physics=physx` 会挡住 VariantSwitcher** → Newton 下可能仍加载 physx 轨，mujoco 的 mute 根本不进舞台。W2 根已去掉 authored Physics 默认（与手资产策略 B 一致）；**不要靠 env 覆盖**。
2. **Newton `import_usd` 对过滤的限制**：`filteredPairs` 必须在碰撞 SHAPE 上且两端都在 `path_shape_map`；刚体级无效。实践中 instanceable + payload 组合还常导致 Stage Property **看不到** filteredPairs。  
   **W2/M6 现行 Newton mute**：对 NonDexHand 碰撞 geom 写 **`physics:collisionEnabled=0`**（与 o7 拇指 CMC mute 同机制，Property 可见）。保留轮子碰撞接地；手不在 mute 列表。文件：`body_self_collision_mute_newton.usda` / `arm_self_collision_mute_newton.usda`。

#### 双轨文件对照（已对齐）

| 层 | PhysX | Newton / mujoco |
|----|-------|-----------------|
| W2 自碰开关 | `enabledSelfCollisions=1` | `newton:selfCollisionEnabled=1` |
| W2 静音 | `body_self_collision_mute.usda`（CollisionGroup） | `body_self_collision_mute_newton.usda`（**collisionEnabled=0**，保留轮子） |
| M6 自碰开关 | self-col=1 | `newton:selfCollisionEnabled=1` |
| M6 静音 | `arm_self_collision_mute.usda`（CollisionGroup） | `arm_self_collision_mute_newton.usda`（**collisionEnabled=0**） |
| EE excludes | `dexhand_self_collision.usda`（PhysX 组 excludes） | 手 mesh **不 mute** → 手指自碰 |

**禁止**把 Newton shape mute 挂进 physx 轨；**禁止**为 Newton 去改手 `physics.usda` FilteredPairs（拇指见 N.7）；**禁止**在临时 env 里盖 Physics 当长期方案。

#### 验收

1. PhysX 回归：握拳不穿模、机体不抖  
2. Newton：Reload 后确认根 `Physics=mujoco`；Play 机体不较劲；手指互碰顶住  
3. Property：看 `…/Left_Arm/link1/collisions/mesh_0/cylinder` 的 **`physics:collisionEnabled = False`**（不是 filteredPairs）  
4. 若仍较劲：跑 `W2/env/newton_verify_mute.py`（运行时强制 mute + 打印 Physics variant / missing paths）  
5. `nconmax` 警告：NonDexHand 假接触过多时会出现；mute 生效后接触数应降下来。勿在临时 env 里长期盖参数。  
