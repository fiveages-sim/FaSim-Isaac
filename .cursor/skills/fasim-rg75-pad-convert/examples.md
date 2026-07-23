---
name: fasim-rg75-pad-convert-examples
---

# RG75 Pad Convert — Examples

## Example: heavy_carry（完整路径）

**输入：** Asset Transformer 导出的 `RG75_Heavy_Left` / `RG75_Heavy_Right` 整爪 USDA。  
**输出：** `robots/grippers/Jodell/RG75/payloads/Pad/heavy_carry/` + `Pad=heavy_carry`。

### 1. 审计结论

| 项 | 发现 |
|----|------|
| 根/底座 | `Jodell_RG75` + `flange` → 映射到 `RG75` + `gripper_base` |
| 运动学 | 固定 tip 在 base；运动 tip 在 `right_finger`；`left_finger` 占位 |
| 左右 | 左右臂 CAD 不同（`bxz101*` vs `bxz102*` 系）；不能 scale=-1 |
| 错误 parenting | Left 源里固定 tip 曾挂在 `flange`；按设计应在 base（固定）或纠正到 finger |

### 2. 抽出

从各侧 Transformer `geometries.usd` / `instances` / `materials` bake 到：

```
Pad/heavy_carry/left/{geometries,instances,materials,Textures,pad_left,pad_right}
Pad/heavy_carry/right/...
```

`pad_*.usda`：`root/mesh` → instances；保留 mesh `rotate` + `scale=0.001`。

### 3. Mount：`heavy.usda`

- `gripper_base/pad_left`：nested `arm` → `./left|right/pad_left.usda` + mount TF  
- `right_finger/pad_right`：nested `arm` → `./left|right/pad_right.usda`  
- `left_finger`：`visibility=invisible`（含 visuals）  
- 清 `simulationOwner`；可选 `PadMaterials` 摩擦

默认 `arm=left`；右臂靠 Side sticky。

### 4. 登记

`RG75.usda`：

- `Pad` 增加 `"heavy_carry"` → `@./payloads/Pad/heavy_carry/heavy.usda@`
- `prepend variantSets = ["Physics", "flange", "side", "Pad", "Sensors"]`

`Side/left.usda` & `right.usda`：sticky `pad_left`/`pad_right` 的 `arm`。

### 5. 物理

`Physics/mujoco.usda`：对 `.../pad_*/mesh` 写 collision overs（剥 PhysX-only schemas）。

### 6. 曾失败的分叉（不要重复）

| 尝试 | 结果 |
|------|------|
| 旧 component `pad_*.usdc` + 矩阵对齐 | 左臂错位 → 回退 |
| `Pad=heavy_carry_left/right` 两个入口 | UX 差 → 合并 + sticky |
| 整 RG75 相对 TCP Rz180 | 其他 Pad 崩 → 禁止 |
| pad 局部补 Rz180 救右臂 | 效果差 → 回退；朝向交父机/EE |
| W2 非 payload 修改 | 违反约束 → 仅 sticky/payload |

### 7. 测试 env

`robots/humanoid/FiveAges/W2/env/lift_box.usda`：双臂 `EE=RG75`，`Pad=heavy_carry`（side 由臂 sticky）。

### 8. 收尾

删除临时 `RG75_Heavy_Left` / `RG75_Heavy_Right`；在 `robots` 子模块 commit（父仓指针另推）。

---

## Example: load_type1（原 PTC，type1-like，L/R 臂同 CAD）

**输入：** `RG75_PTC_Left` / `RG75_PTC_Right`。  
**输出：** `payloads/Pad/load_type1/` + `Pad=load_type1`（用途名；doc 注明 legacy PTC）。

### 审计

| 项 | 结果 |
|----|------|
| 运动学 | 双指 `pad_ptc` 均在 `*_finger` 下 → mimic / type1-like |
| L vs R mesh hash | `Mesh_0` / `Mesh_0_003` 等 **全部相同** |
| L vs R `pad_ptc` 块 | **字节级相同** |
| 差异 | 基本只有文档路径、相机、finger rest 微差；关节名靠 `side` |
| 右指软垫 | 源内可有 `scale=(-1,1,1)`；PhysX 可保留源 TF |

### 做法

- 只从 **Left** bake：`geometries`（`Mesh_0`, `Mesh_0_003`）+ `instances` + 三色 `materials`。
- `pad_left.usda` / `pad_right.usda`：子件 TF 不同（非单 `pad.usdc`）。
- `load_type1.usda`：容器 `rotateZYX (-90,0,0)` / `(-90,-180,0)`；清 `simulationOwner`；藏 `finger_support`。
- **无** nested `arm` / Side sticky。
- 碰撞：Pad `instances` + 可选共享 `mujoco` heavy overs（与已提交 heavy 一致）；**PhysX 验收**。
- 右软尖可保留源 TF（含指间负 scale）；勿为双臂新造 `scale=-1`。

### Skill 缺口（已回写）

- 必须先做 L/R mesh+pad 块比对。
- type1-like 仍可能需要双 tip 包。
- **PhysX 为主验收**；Newton tip 问题不在本 skill 范围。
- 校验用 session layer，勿改默认 `Pad=none`。

---

## Example: load_type2（Track C — 人工 cube 碰撞 + 双 usdc）

**输入（人已做好）：** `pad.usdc`、`pad_right.usdc`（各含 `/root/collision/Cube*`，`boundingCube`）。  
**输出：** `load_type2.usda` 分挂左右；nested `arm` + Side sticky；无 `pad_mirror`、无 mount 负 scale。

### 任务澄清（易混点）

| 现象 | 原因 | 正确做法 |
|------|------|----------|
| 左右 Stage 同时出现 collision | 两指都 payload 同一 `pad.usdc` | 右指改 `@./pad_right.usdc@` |
| mesh 与 cube 都碰 | mount 曾给视觉 mesh 开 convexDecomposition | 关 mesh 碰撞；只用 cubes |
| 需要镜像 | 形状不规则，不能靠旋转 | 人 Save As 第二份 usdc 手调；不要 AI bake |

### Agent 已做适配

- 左指 → `pad_right.usdc` + `(-90,0,0)`；右指 → `pad.usdc` + `(-90,180,0)`；`scale=(1,1,1)`
  （`pad_right.usdc` ≈ 旧左指 `pad.usdc*scale(-1,1,1)`；勿按文件名挂右指）
- `arm=left/right` **同内容**（不按 side 互换 tip；scanner 在 base 上，互换会把 pad 相对 scanner 翻面）
- 视觉 mesh 无碰撞；已删 `pad_mirror.usdc`

细则与 GUI 前置步骤：[track-c-simplified-collision.md](track-c-simplified-collision.md)。

---

## Example: 下一款「对称双指」pad（预期流程）

1. **先** L/R mesh hash + pad 块 diff。  
2. 两 tip 均在 finger 下 → type1-like；相同则只 bake 一侧。  
3. 若左右 tip **内部** xform 不同 → `pad_left`+`pad_right`；相同 → 可单 `pad.usdc`。  
4. 登记 `Pad=<name>`；**PhysX** 下验开合；不改 Side 关节逻辑。

---

## Example: 仅右臂需要专用 CAD

仍用 **一个** `Pad=<name>` + nested `arm`；`right/` 放专用几何，`left/` 放左臂几何。  
Side sticky 负责选择；避免只做 `Pad=<name>_right`。
