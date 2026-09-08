---
name: fasim-dexhand-asset
description: >-
  Build FaSim-Isaac unified dexterous-hand USD assets (LinkerHand o6/o7,
  BrainCo Revo1/Revo2, etc.) from separate left/right USDA imports: confirm
  mirror axis (X like o6 vs Y like o7) before conversion; PhysX-safe mirror
  bake (root scale always (1,1,1)); Hand/left|right morphology; Side variants
  with left_hand_/right_hand_ rename; Physics variants; validated sim baseline
  (maxJointVelocity=500000; drive D=0.0005 K=0.005 F=1.68 force; self-collision +
  FilteredPairs; collider friction). Warn AssemblerFixedJoint must match mount TF;
  never ship build scripts or raw *_left/*_right trees. Does NOT write arm EE mounts.
---

# FaSim Dexterous Hand Asset

从**左右手各自**的 ROS2 导入 + Asset Transformer USDA，做成与
`robots/dexhands/LinkerHands/o7/` / `o6/` 同构的**单一入口**资产。

## 交付状态

| 型号 | 结构 Side L/R | 关节基线 | 自碰+Filter | 碰撞摩擦 | 备注 |
|------|---------------|----------|-------------|---------|------|
| **o6** | ✅ | ✅ D0.0005/K0.005/F1.68 + maxVel | ✅ | ✅ | **仿真参考手** |
| **o7** | ✅ | ✅ 同套 | ✅ 保留捏合 | ✅ | 已对齐 |

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
| `physxJoint:maxJointVelocity` | **500000** | **P0 冻结**；勿用真机 70–260「防尖峰」 |
| 主动 drive | type=`force`，**D=0.0005 K=0.005 F=1.68**，targetPos=0 | 超软基线；手感再调（先 F 再 K） |
| Mimic | `dampingRatio=0` `naturalFrequency=0` `offset=0` | 硬耦合；勿随意加 compliance |
| 自碰 | **On** + `PhysicsFilteredPairsAPI` | 滤假接触，**保留**拇指 tip↔指尖捏合 |
| 摩擦 | collider `PhysicsMaterialAPI` μs=1.5 μd=1.2 rest=0 | Inspire 官方无此项 |
| 碰撞 | mesh + `convexHull`；det&gt;0 | |
| Solver | pos=64 · vel=1 | 暂不折腾 |

```usda
float drive:angular:physics:damping = 0.0005
float drive:angular:physics:maxForce = 1.68
float drive:angular:physics:stiffness = 0.005
float drive:angular:physics:targetPosition = 0
uniform token drive:angular:physics:type = "force"

float physxJoint:maxJointVelocity = 500000
```

### Drive 参数怎么调（force 型）

力矩近似：`τ ≈ K·(q_target − q) − D·q̇`，再被 `maxForce` 截断。

| 参数 | 作用 | 调大 | 调小 |
|------|------|------|------|
| **stiffness (K)** | 拉回目标角的「弹簧」硬度 | 更跟手、接触后不易被顶偏 | 更软、更让、易耷拉 |
| **damping (D)** | 角速度阻尼，耗散振荡 | 更稳、少抖；过大则发黏 | 更快响应；过小易颤 |
| **maxForce (F)** | 驱动能输出的最大力矩 | 更有「力气」、能顶住物体 | 易饱和→看起来没劲 |
| **type** | `force` / `acceleration` | 统一用 **force**（与运控/质量一致） | 勿左右混用 |
| **targetPosition** | 位置目标（度，USD 习惯） | — | 资产默认 0；运行时由控制器写 |

**推荐手感区间（灵巧手，非机械臂）：**

- **当前交付（超软稳）**：`K=0.005 D=0.0005 F=1.68` → 不易顶飞；偏软可后调  
- 中等合规：先加 **F**，再小加 **K**；**D** 大约 `0.05–0.2×K`，抖再加大 D  
- 机械臂级（`K` 上百）不适合手指  

**与 `maxJointVelocity` 分工：** 限速管炸飞；**软硬/力气是 K/D/F**。

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
