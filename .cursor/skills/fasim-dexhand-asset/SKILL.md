---
name: fasim-dexhand-asset
description: >-
  Build FaSim-Isaac unified dexterous-hand USD assets (LinkerHand o6/o7,
  BrainCo Revo1/Revo2, etc.) from separate left/right USDA imports: confirm
  mirror axis (X like o6 vs Y like o7) before conversion; PhysX-safe mirror
  bake (root scale always (1,1,1)); Hand/left|right morphology; Side variants
  with left_hand_/right_hand_ rename; Physics variants; validated sim baseline
  (maxJointVelocity=500000 frozen; G1 hold D=0.06 K=0.1 F=30 / thumb
  D=0.08 K=0.12 F=50 + jointFriction=0.5; μs=2.0 contactOffset=0.002;
  distal CCD; self-collision + FilteredPairs). Warn AssemblerFixedJoint must
  match mount TF; never ship build scripts or raw *_left/*_right trees.
  Does NOT write arm EE mounts.
---

# FaSim Dexterous Hand Asset

从**左右手各自**的 ROS2 导入 + Asset Transformer USDA，做成与
`robots/dexhands/LinkerHands/o7/` / `o6/` 同构的**单一入口**资产。

## 交付状态

| 型号 | 结构 Side L/R | 关节基线 | 自碰+Filter | 碰撞摩擦 | 备注 |
|------|---------------|----------|-------------|---------|------|
| **o6** | ✅ | **G1 hold 已同步 o7** | ✅ | ✅ μ2.0+contactOffset | W2:`dexhand_o6_new`；单手:`o6/env/test` |
| **o7** | ✅ | **G1 完全通过** | ✅ 保留捏合 | ✅ | W2:`dexhand_o7_new` 抓瓶不穿不粘可抬 |

**黄金参考：**

- **结构 / Side / 挂载**：`LinkerHands/o6/`、夹爪 `grippers/Jodell/RG75/`
- **官方自碰写法**：`dexhands/Inspire/module_5_end-checkpoint_3/`（开自碰 + `PhysicsFilteredPairsAPI`）
- **关节 PhysX（o6/o7 已跑通）**：§0.1（含 K/D/F 调参说明）

**资产目录只保留交付物**（入口 USDA + `payloads/` + `Textures/`）。  
禁止放入：`o7_left/`、`o7_right/`、`_*.py` 构建脚本、`__pycache__/`、`transform_report.json`。算法写在 [reference.md](reference.md)。

**待处理批次：**

| 源（左右分文件夹） | 目标统一文件夹 |
|--------------------|----------------|
| `LinkerHand_o6_*` | `LinkerHands/o6`（✅） |
| `LinkerHand_o7_*` | `LinkerHands/o7`（✅） |
| `BrainCo_Revo1_*` | `BrainCo/Revo1` |
| `BrainCo_Revo2_*` | `BrainCo/Revo2` |

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
   - [ ] Y 镜像（o7）：PhysX 安全烘焙（§2B / reference §Y）；根 (1,1,1)；Side/right 关节 Y 镜像 + X/Z 轴 ×Ry(180°)
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
    Hand/left|right/        # base, instances, geometries, materials, robot
    Side/default|left|right.usda
    Physics/physics|physx|mujoco|none.usda
    Robot/  Sensor/
```

---

## 4. Side variant

| Side | 形态 | 关节 |
|------|------|------|
| default | 左 | 无前缀 |
| left | 左 | `left_hand_*` |
| right | 右 | `right_hand_*`；入口 `delete` 左 base → `Hand/right` |

Y 镜像 right：关节/质量 localPos/Rot 同步 Y 镜像；`axis=X|Z` 再 ×Ry(180°)（拇指 joint2/3）。

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
资产目录无 *_left/*_right、无 _*.py、无 __pycache__
挂臂：FixedJoint = 挂载 TF；Play 不跳
接触：握拳/指垫互碰不飞；可抓桌上物体（物体侧也要摩擦）
```

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
