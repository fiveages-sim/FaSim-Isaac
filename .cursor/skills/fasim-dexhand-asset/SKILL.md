---
name: fasim-dexhand-asset
description: >-
  Build FaSim-Isaac unified dexterous-hand USD assets (LinkerHand o6/o7,
  BrainCo Revo1/Revo2, etc.) from separate left/right USDA imports: bake
  PhysX-illegal collision scale=(-1,*), Hand/left|right morphology trees,
  Side=default|left|right with left_hand_/right_hand_ joint rename, Physics
  variants, and optional FilteredPairs. Use when converting dexhands, fixing
  mirror collision flicker, or batching new hand folders under robots/dexhands.
  Does NOT configure manipulator EE mounts (arm mount is a separate step).
---

# FaSim Dexterous Hand Asset (o6-style)

从**左右手各自**的 ROS2 导入 + Asset Transformer USDA，做成与
`robots/dexhands/LinkerHands/o6/` 同构的**单一入口**资产：

- 入口：`{Brand}/{name}/{Name}.usda`（例：`LinkerHands/o6/LinkerHand_o6.usda`）
- `Side=default|left|right`：形态 + 关节改名
- 碰撞体禁止负 scale（PhysX）；右手镜像须烘焙
- **不**写机械臂 EE 挂载（Marvin / W2 挂载另做）

**黄金参考（先读再仿）：** `robots/dexhands/LinkerHands/o6/`  
结构参考夹爪：`robots/grippers/Jodell/RG75/`（Side / Physics / payload）  
关节改名参考：`manipulators/Tianji/Marvin_M6_CCS/payloads/Side/left.usda`、RG75 `payloads/Side/left.usda`

**待处理批次（可批量）：**

| 源（左右分文件夹） | 目标统一文件夹 |
|--------------------|----------------|
| `LinkerHand_o6_left` / `_right`（或等价） | `LinkerHands/o6` |
| `LinkerHand_o7_*` | `LinkerHands/o7` |
| `BrainCo_Revo1_*` | `BrainCo/Revo1`（按品牌归类） |
| `BrainCo_Revo2_*` | `BrainCo/Revo2` |

命名以用户指定为准；原则：**按牌子分类**，一个型号一个入口 USDA。

---

## 0. 问诊（先问再改）

```
【DexHand 问诊】
1. 左右源路径？（*_left / *_right 入口 usda）
2. 目标品牌/型号文件夹名？入口 prim 名？（例 LinkerHand_o6）
3. 关节前缀？默认 left_hand_ / right_hand_（对齐 fa_w2_ws o6.side.yaml）
4. default Side 用哪只手形态？通常 = 左手
5. 驱动关节列表 + mimic 映射？（leader → follower, gearing）
6. mimic gearing 数值？（从源 physx 读；**勿改** dampingRatio/naturalFrequency）
7. 本轮是否做 FilteredPairs？[ ] 暂缓 [ ] 做（需开 sim 扫）
8. 做完是否删除旧左右文件夹？[ ] 是 [ ] 否
```

---

## 0.1 关节参数铁律（禁止改）

制作 / Side 改名 / 入口 overs **不得**覆盖或“优化”下列 PhysX 参数；改了手会坏。
只允许改名、改 body 路径、改 mimic 的 `referenceJoint` 指向；`gearing` 从源资产原样搬。

### 主动关节（有 Drive 的 revolute）

| 项 | 固定值 |
|----|--------|
| Drive type | `force` |
| Max Force | `10` |
| Target Position | `0` |
| Target Velocity | `0` |
| Damping | `1` |
| Stiffness | `3` |

```usda
float drive:angular:physics:damping = 1
float drive:angular:physics:maxForce = 10
float drive:angular:physics:stiffness = 3
float drive:angular:physics:targetPosition = 0
float drive:angular:physics:targetVelocity = 0   # 若层上有则保持 0
uniform token drive:angular:physics:type = "force"
```

### Mimic 关节（PhysxMimicJointAPI）

| 项 | 固定值 |
|----|--------|
| Damping Ratio | `0` |
| Natural Frequency | `0` |
| Offset | `0` |
| Gearing | **保留源资产值**（勿统一改） |
| Reference Joint | 改名前缀后的 leader 路径 |

```usda
float physxMimicJoint:rotY:dampingRatio = 0
float physxMimicJoint:rotY:naturalFrequency = 0
float physxMimicJoint:rotY:offset = 0
# gearing / referenceJoint：从源 physx 拷贝后只改 joint 名路径
```

**禁止**：把 mimic 写成 `dampingRatio=0.005` / `naturalFrequency=25`（导入默认值）；禁止把主动关节刚度阻尼改回 physics.usda 里的小数值。

---

## 1. 总流程（不可跳步）

```
- [ ] A. 审计左右源：prim 名、关节名、碰撞负 scale、mimic、Physics 层
- [ ] B. 右手（及任何 det=-1 碰撞）烘焙 X-flip 网格；碰撞 scale→(1,1,1)；视觉可留 (-1,*)
- [ ] C. 建目标文件夹 + Hand/left + Hand/right（改名统一 prim）
- [ ] D. 共享 Physics（默认用左手 frames）+ Robot/Sensor 壳
- [ ] E. Side default/left/right：改名关节；right 切换 Hand/right 引用
- [ ] F. 入口 USDA：Side 最强，Physics 次之；default 增益 overs 只写无前缀关节
- [ ] G. USD 组合校验（形态 X 符号、关节 active、碰撞 det=+1）
- [ ] H.（可选）FilteredPairs 方案见 §6；本 skill 默认可只给方案不写数
- [ ] I. 不写臂 EE 挂载
```

---

## 2. 碰撞负 scale（必做，否则闪烁 / 丢碰撞）

### 现象

右手镜像常用 `xformOp:scale = (-1, 1, 1)` 复用左手 mesh。  
**PhysX 无法烹饪 world 行列式为负的 mesh collider** → flange/拇指等碰撞消失 + Physics Task 闪烁。

### 修法（o6 已验证）

1. 找出**碰撞**路径上 `min(scale)<0` 的 instance（视觉可保留负 scale）。
2. 在该侧 `geometries.usd` 建 `mesh_*_xflip`：points.x 取反、法线.x 取反、逐面反转 winding、更新 `extent`；**内部 Mesh 子节点名与源同名**（保证 instances 的 `over "mesh_N"` 仍命中）。
3. `instances.usda` 里**仅碰撞** instance（常 `mesh_*_1`）改引用 `.../mesh_*_xflip`。
4. `base.usda` 碰撞 xform：`scale = (1,1,1)`，保留原 orient/translate。  
   数学：`R * S * p` ≡ `R * p_xflip`。

### 验收

- 碰撞 prim world `det > 0`
- 同 link 下 visual vs collision 点变换误差 ≈ 0
- Play PhysX：碰撞可见、无持续 Physics Task 报错

工具：Isaac `kit/python` + `omni.usd.libs`（见 `fasim-usd-bake-scale` 的 PYTHONPATH 写法）。  
**不要**对碰撞只用「去掉 -1 不烘焙」——几何会偏到错误半边。

---

## 3. 目标目录结构（对齐 o6）

```
{Brand}/{name}/
  {Name}.usda                 # 唯一入口
  Textures/                    # 可选根级；Hand 内也可自带
  payloads/
    base.usda                 # 薄层 → Hand/left/base.usda
    Hand/
      left/                   # base, instances, geometries, materials, robot, Textures?
      right/                  # 同上（已修碰撞）
    Side/
      default.usda            # 空壳即可
      left.usda               # deactivate 无前缀；def left_hand_*
      right.usda              # deactivate 无前缀；def right_hand_*（用右手 joint frames）
    Physics/                  # 自左手改 prim 名：physics, physx, mujoco, none
    Robot/  Sensor/           # 可先 none/robot 壳
```

原则：

- `defaultPrim` / 根 Xform 名统一为 `{Name}`（去掉 `_left/_right`）
- `payloads/base.usda` **不要**再复制一份 14MB+ geometries；redirect 到 `Hand/left`
- 材质里 `@../Textures/pbr.mdl@` 若文件在 `Hand/{side}/Textures/`，改为 `@./Textures/pbr.mdl@`

---

## 4. Side variant（关节改名 + 形态）

### 语义（对齐运控 `o6.side.yaml`）

| Side | 形态 | 活动关节名 |
|------|------|------------|
| `default` | 左手 | 无前缀 `index_joint` … |
| `left` | 左手 | `left_hand_*`；无前缀 `active=false` |
| `right` | 右手 | `right_hand_*`；无前缀 `active=false` |

前缀格式：`{side}_hand_{joint}`（`side`∈`left|right`）。  
与 `fa_w2_ws/.../o6.side.yaml` 的 `{side_}hand_*` 一致。

### 入口 variant（Side 强于 Physics）

```usda
prepend variantSets = ["Side", "Physics"]
append variantSets = ["Robot", "Sensor"]
variants = { string Side = "default", string Physics = "physx", ... }

variantSet "Side" = {
  "default" ( prepend payload = @./payloads/Side/default.usda@ ) {}
  "left"    ( prepend payload = @./payloads/Side/left.usda@ ) {}
  "right" (
    delete references = @./payloads/base.usda@
    prepend references = @./payloads/Hand/right/base.usda@
    prepend payload = @./payloads/Side/right.usda@
  ) {}
}
```

`Side=right` **必须**在入口层 `delete` 左手 base、`prepend` 右手 base，避免左右网格叠载。

### Side/left|right.usda 内容

仿 RG75 / Marvin Side：

1. 对每个默认关节：`over "index_joint" (active=false) {}`
2. `def PhysicsRevoluteJoint "left_hand_index_joint"`（或 right_）  
   - 从对应侧 `physics.usda` 拷 body0/1、localPos/Rot、limits  
   - 主动关节 Drive：**强制** §0.1（damping=1, stiffness=3, maxForce=10, type=force）  
   - 叠 `PhysxJointAPI`；mimic 加 `PhysxMimicJointAPI:rotY`，`referenceJoint`→同前缀 leader；**dampingRatio/naturalFrequency/offset=0**，gearing 用源值  
3. `Side=right`：另 over 右手质量（`PhysicsMassAPI`），因共享 Physics 层多为左手质量；Side 强于 Physics 时可盖住  
4. **禁止**在入口根层写 `left_hand_*` / `right_hand_*` 的空 `over`（Kit 会话残留会在 default 冒幽灵关节）

### Mimic / Drive

见 §0.1。Side 层覆盖时也必须用同一套，不得回退到导入默认的 mimic NF=25 等。

---

## 5. Physics / Robot / 入口增益

- `Physics/physics.usda`：刚体、关节、限位（路径改为统一 prim）  
- `Physics/physx.usda`：`subLayers = [@./physics.usda@]`；无前缀关节的 PhysX / mimic（供 `Side=default`）  
- `Physics/mujoco.usda`：可先占位或后续用 `fasim-robot-mujoco-physics`  
- 入口对 **default 无前缀驱动关节** 写 drive overs（用户增益）  
- `Robot`：可先 `none`；真机仿真图多在**臂**上合并，手资产不必强开独立 ros2_graph

---

## 6. 碰撞对 FilteredPairs（遗漏项 · 方案）

官方（Isaac Sim 6.0）：[Tutorial 4: Collider Pairs](https://docs.isaacsim.omniverse.nvidia.com/6.0.0/openusd_tuning_tutorials/tutorial_04_collider_pairs.html)、[Robot Self-Collision Detector](https://docs.isaacsim.omniverse.nvidia.com/6.0.0/robot_setup/ext_isaacsim_robot_setup_collision_detector.html)。

o6 现状：`physxArticulation:enabledSelfCollisions = 0` → 自碰全关，稳但不真实。灵巧手官方教程路径：

1. 在 `physx.usda` 将 **Self Collisions = On**（articulation root）  
2. Tools → Robotics → **Robot Self-Collision Detector**，静止位姿扫重叠对  
3. 对「静位重叠 / 会抖爆」的 link 对勾选 Filtered Pair → 写入 `UsdPhysics.FilteredPairsAPI`  
4. **写在 `payloads/Physics/physics.usda`（或 physx）**，不要写进 base 几何层  
5. 左右形态 link 名相同则可共用一份 filter；若左右碰撞包络差很大，Right 可在 `Side/right.usda` 补 over  
6. 验收：Play 握拳不炸、指尖仍能碰物体；不过滤「需要的」指-指 / 指-物接触

```usda
over "flange" ( prepend apiSchemas = ["PhysicsFilteredPairsAPI"] )
{
    prepend rel physics:filteredPairs = </Hand/index_proximal>
}
```

**本 skill 默认**：结构做完后给出上述方案；数值对需用户在 Isaac 里扫完再提交列表再写入。

---

## 7. 批量处理

对表中每个型号独立跑 §1 流程；脚本化时可共享：

- 负 scale 扫描 + xflip 烘焙  
- prim 重命名 `*_left|right` → `{Name}`  
- Side 关节模板（JOINTS / DRIVEN / MIMIC / PREFIX）

每型号 **MIMIC/限位/增益不可盲拷 o6**，从该型号源 physics/physx 提取。

---

## 8. 校验清单

```
Side=default: 左手形态；仅 无前缀 关节 active；碰撞 det>0
Side=left:    左手；left_hand_* active；无前缀 inactive
Side=right:   右手（如 index.x 符号翻转）；right_hand_* active；无前缀 inactive
入口根层无 left_hand_/right_hand_ 幽灵 over
不修改机械臂除用户明确要求外的文件
```

---

## 9. 明确不做

- 机械臂 `EE` / tcp 挂载、AssemblerFixedJoint、臂 Side sticky（另任务；参考 Marvin `payloads/EE/LinkerHand_o6.usda`）  
- 擅自改运控 yaml（只保证关节名可对齐 `o6.side.yaml` 模式）  
- 把失败的 `payloads/Mesh/*` 实验结构当参考（以 `Hand/left|right` + `Side` 为准）

## 更多

见同目录 [reference.md](reference.md)。
