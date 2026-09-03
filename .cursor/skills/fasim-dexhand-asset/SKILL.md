---
name: fasim-dexhand-asset
description: >-
  Build FaSim-Isaac unified dexterous-hand USD assets (LinkerHand o6/o7,
  BrainCo Revo1/Revo2, etc.) from separate left/right USDA imports: confirm
  mirror axis (X like o6 vs Y like o7) before conversion; PhysX-safe mirror
  bake (root scale always (1,1,1)); Hand/left|right morphology; Side variants
  with left_hand_/right_hand_ rename; Physics variants; contact baseline from
  o7 (convexHull, mimic 0/0, finger K=5/F=20). Warn AssemblerFixedJoint must
  match mount TF; never ship build scripts or raw *_left/*_right trees in the
  asset folder. Does NOT write arm EE mounts (separate step).
---

# FaSim Dexterous Hand Asset

从**左右手各自**的 ROS2 导入 + Asset Transformer USDA，做成与
`robots/dexhands/LinkerHands/o7/`（接触参考）/ `o6/`（结构参考）同构的**单一入口**资产。

## 交付状态

| 型号 | 结构 Side L/R | 接触交互（轻按桌 / 抓方块） | 备注 |
|------|---------------|------------------------------|------|
| **o7** | ✅ 完成 | ✅ 验收参考 | PhysX 安全 Y 烘焙；mesh `convexHull` |
| **o6** | ✅ 已有 | 🔄 对齐 o7 接触基线 | 恢复 mesh 碰撞；禁止胶囊简化当交付 |

**黄金参考：**

- **结构 / Side / 挂载套路**：`LinkerHands/o6/`、夹爪 `grippers/Jodell/RG75/`
- **接触物理（轻柔交互）**：`LinkerHands/o7/` —— 轻压桌不飞、无吸附、可抓动态方块；大力戳指仍可能抽搐并拽倒机器人（可接受上限）

**资产目录只保留交付物**（入口 USDA + `payloads/` + `Textures/`）。  
禁止放入：`o7_left/`、`o7_right/`、`_*.py` 构建脚本、`__pycache__/`、`transform_report.json`。算法写在 [reference.md](reference.md)。

**待处理批次：**

| 源（左右分文件夹） | 目标统一文件夹 |
|--------------------|----------------|
| `LinkerHand_o6_*` | `LinkerHands/o6`（已有；接触对齐 o7） |
| `LinkerHand_o7_*` | `LinkerHands/o7`（✅ 完成） |
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
6. 驱动 / mimic：从源读；接触基线默认对齐 o7（§0.1）
7. 挂臂后拇指重力下垂？可加大拇指 K/F（o7 曾用 K=50/F=80/D=5）
8. FilteredPairs？[ ] 暂缓 [ ] 做
9. 做完删除源左右文件夹？[ ] 是 [ ] 否（是则删干净，勿留在资产目录）
```

**挂载（本 skill 不写臂 EE，联调必查）：**

1. `AssemblerFixedJoint` 的 `localPos0/localRot0` **=** 挂载 `translate/orient`（否则 Play 跳变）。
2. 右手根 / 挂载 **禁止** 负 scale；形态必须已烘焙到根 `(1,1,1)`。

---

## 0.1 接触物理基线（以 o7 为准）

目标：**轻柔交互**（轻按静物、抓动态小物体）像 o7；不要求大力戳指零起飞。

| 项 | o7 / 推荐 | 禁止 |
|----|-----------|------|
| 碰撞几何 | **mesh** + `PhysicsMeshCollisionAPI` `approximation = convexHull` | 胶囊/方盒简化当交付；`convexDecomposition` 作默认（易尖点冲量） |
| 碰撞 scale | world det &gt; 0（镜像须烘焙） | 碰撞路径负 scale |
| Mimic | `dampingRatio=0` `naturalFrequency=0` `offset=0`；gearing 源值 | NF=20/25、dampingRatio=0.005 |
| Articulation | `enabledSelfCollisions=0`；solver **64 / 1**；sleep/stab 同 o7 | 乱加 per-link `maxLinearVelocity` 当主手段 |
| 四指主动 drive | type=force，**D=1 K=5 F=20** | 靠把 K 拧到极软“防飞” |
| 拇指主动 drive | type=force，**D=1 K=10 F=30**（抗重力可再加） | 同上 |
| 质量 | 保持源 mesh 质量量级（克级指节） | 为防飞把指节质量抬到几十克当唯一手段 |

```usda
# 四指主动
float drive:angular:physics:damping = 1
float drive:angular:physics:maxForce = 20
float drive:angular:physics:stiffness = 5
uniform token drive:angular:physics:type = "force"

# 拇指主动（基线）
float drive:angular:physics:damping = 1
float drive:angular:physics:maxForce = 30
float drive:angular:physics:stiffness = 10

# Mimic
float physxMimicJoint:rotY:dampingRatio = 0
float physxMimicJoint:rotY:naturalFrequency = 0
float physxMimicJoint:rotY:offset = 0
```

**大力戳指仍抽搐并拽倒机器人**：o7 亦有；属可接受上限，优先查接触几何/穿透修正，而不是先把驱动拧死。

历史备注：早期 o6「铁律 K=3/F=10」已被 **o7 接触基线** 取代用于交互调参；新转换默认用本节表。

---

## 1. 总流程

```
- [ ] A. 审计左右源：prim、关节、碰撞负 scale、mimic、Physics
- [ ] A2. 确认镜像轴（问诊 §5）
- [ ] B. 按轴烘焙碰撞 / 形态；根 scale=(1,1,1)；碰撞 det>0
- [ ] C. 建 Hand/left|right + 共享 Physics/Robot/Sensor
- [ ] D. Side default|left|right（改名；right 切 Hand/right；Y 轴时关节 Y 镜像 + X/Z 纠转向）
- [ ] E. 入口 USDA；接触参数对齐 §0.1
- [ ] F. 校验：拇指解剖侧、关节 active、碰撞 det、Play 挂载不跳
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
- 挂臂 Play：位姿不跳；轻压桌/抓方块接近 o7  

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

## 5–6. Physics / FilteredPairs

见 §0.1；FilteredPairs 仍可选（自碰默认关）。细节同前版教程链接。

---

## 8. 校验清单

```
Side L/R/default 形态与关节前缀正确
碰撞 mesh convexHull；无胶囊交付；det>0
mimic 0/0；drive 对齐 §0.1
资产目录无 *_left/*_right、无 _*.py、无 __pycache__
挂臂：FixedJoint = 挂载 TF；Play 不跳
接触：轻压桌不飞、无明显吸附（对标 o7）
```

---

## 9. 明确不做

- 在本 skill 里写满机械臂 EE（只给挂载约束）  
- 把胶囊/方盒简化碰撞当最终交付  
- 把构建脚本留在 `robots/dexhands/...` 资产树内  

## 更多

[reference.md](reference.md) — Y 烘焙步骤、Isaac 片段、o6↔o7 接触对照。
