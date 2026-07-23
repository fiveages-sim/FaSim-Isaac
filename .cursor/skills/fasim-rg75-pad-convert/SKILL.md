---
name: fasim-rg75-pad-convert
description: >-
  Convert or mount Jodell RG75 finger pads onto robots/grippers/Jodell/RG75 Pad
  variants. Tracks: A Asset-Transformer bake; B thin tip-USDC mount (rotation OK);
  C human cube-collider tip USDCs (no mount scale=-1). Use for heavy_carry, type1,
  load_type1, load_type2, detector_type1, scanner sensors. ALWAYS ask the intake
  template before editing; stay inside RG75 Pad/Sensor payloads unless user says
  otherwise.
---

# FaSim RG75 Pad Convert

把夹指 tip（及可选底座 Sensor）接到
`robots/grippers/Jodell/RG75/` 的 **`Pad` / `Sensors` variant**。

**验收：PhysX（主路径）。** 不负责 Newton/MJWarp tip 调参。

---

## 0. 问诊模板（先问再改）

直接套用下面模板（用户一般门清；把选项说全即可）。缺关键项再追问。

```
【RG75 Pad 问诊】
1. Pad 用途名（snake_case；文件夹 = 入口 usda = Pad= 同名）？
   例：heavy_carry / load_type1 / load_type2 / …

2. 是否配套 gripper_base 上的 Sensor？
   [ ] 无
   [ ] 有 → Sensors 名：________ ；TF 来源：________

3. 输入形态（选一）？
   [ ] A：Asset Transformer 整爪 USDA（左/右或单侧）
   [ ] B：现成 tip .usdc（靠旋转对齐左右指）
   [ ] C：已含 /root/collision/Cube* 的 tip .usdc（不规则则两份镜像）

4. 碰撞（选一；可多选说明）？
   [ ] 暂无 / 后补
   [ ] tip mesh convexDecomposition（或源 usdc 已带）
   [ ] 人工 Cube（boundingCube / convexHull）
   [ ] 其他：________

5. 运动学（选一）？
   [ ] type1-like：双指都随 finger 动
   [ ] heavy-like：一 tip 固定 base + 一 tip 在 finger（占位 finger invisible）

6. 左右机器人臂 tip CAD（选一）？
   [ ] 相同（指间 L/R 可以不同文件）→ 默认不改 Side
   [ ] 不同 → 需要 nested arm；若要改 Side sticky 须你明确说「可以改 Side」

7. Mount TF？
   [ ] 已有参考路径/数值：________
   [ ] 对齐某现有 PhysX 版：________
   [ ] 一起估

8. 验收目标（可多选）？
   [ ] PhysX 开合  [ ] 左右臂外观  [ ] scanner↔pad 相对位姿
   [ ] ROS2 运控  [ ] 其他：________

9. 是否授权改 RG75 入口以外的文件（默认否）？
   [ ] 否（只动 Pad/Sensor payload + 入口追加 variant）
   [ ] 是，允许：Side / Physics/mujoco / 其他：________
```

复述：**Track = ?；将新建哪些路径；入口只追加哪些 variant；默认不动 Side/Physics/父机。** 确认后再改。

---

## 1. Tracks（三选一）

| Track | 输入 | Golden | Side / Physics |
|-------|------|--------|----------------|
| **A — bake** | Transformer 整爪 | `heavy_carry/`、`load_type1/` | **默认不改**；heavy 式要 Side 时须用户明确授权 |
| **B — USDC 薄挂载** | tip usdc + 旋转可对齐 | `type1/`、`detector_type1/` | 不改 |
| **C — 简化碰撞 USDC** | Cube tip usdc；镜像两份 | `load_type2/` | 不改 |

细则：[track-a-bake.md](track-a-bake.md) · [track-b-usdc-mount.md](track-b-usdc-mount.md) · [track-c-simplified-collision.md](track-c-simplified-collision.md)

**选轨**

```
Transformer 整爪？ ──是──► A
tip usdc 且有 Cube 碰撞？ ──是──► C
tip usdc 且旋转可对齐？ ──是──► B
必须镜像且无第二份 usdc？ ──► 请用户先 GUI 做第二份（Track C 前置）
```

**禁止：** mount `scale=-1`；AI `pad_mirror` 主路径；未授权改 Side 却按 side 互换 tip。

---

## 2. 修改边界（极重要）

操作范围：**仅** `robots/grippers/Jodell/RG75/`。  
本 skill **与 M6 / W2 本体无关**——不要去改机械臂/人形机仓库或 EE 挂载来「顺便修 Pad」。

### 默认允许（除非用户另行指定）

| 允许 | 说明 |
|------|------|
| **新建** `payloads/Pad/<purpose>/` | tip 装载 payload（usda/usdc/geometries…） |
| **新建** `payloads/Sensor/<name>/` | 仅当问诊确认有配套 sensor |
| **`RG75.usda` 仅追加** `Pad=` / `Sensors=` variant 项 | 登记入口；见下「默认状态」 |

### 默认禁止（须用户明确点名才可动）

| 禁止 | |
|------|--|
| `payloads/Side/*.usda` | 含 sticky |
| `payloads/Physics/**` | 含 mujoco overs |
| `payloads/flange/**`、关节/执行器层、已有 Pad/Sensor 目录的**改写** | |
| `RG75/` **以外**任何路径 | 含 W2、M6、fa-project-usd 场景（除非用户指定只当 TF 参考只读） |
| 改已有其他 Pad 的文件或行为 | 新 Pad 不得影响 type1/heavy/… |

### 入口文件默认状态（硬约束）

- 只 **追加** variant 条目，不改其它 variant 内容与顺序（保持  
  `["Physics","flange","side","Pad","Sensors"]`）。
- **禁止**把 `Pad=<新名>` / `Sensors=<新名>` 写成默认选中；默认保持 `Pad=none`、`Sensors=none`（或仓库原默认）。
- 校验只用 **session layer**，勿把选择存进金色 `RG75.usda`。

### 其它硬禁

- 改 RG75↔TCP / 根朝向「救」某一 Pad  
- mount `scale` 负分量  
- 视觉 mesh 与 Cube 双开碰撞  
- 本流程 chase Newton tip  

---

## 3. 命名

| 规则 | 例 |
|------|-----|
| 用途名 snake_case | `load_type2` |
| `Pad/<name>/` + `<name>.usda` + `Pad=<name>` 同名 | |
| 旧工程名只写 doc | `ptc` → `load_type1` |
| 已上线名不改 | `type1`、`heavy_carry` |

---

## 4. 标准工作流

```
1. 问诊模板 → 定 Track / Sensor / 授权范围
2. 只读 Golden + RG75.usda 登记写法
3. 新建 payloads/Pad/<purpose>/（+ 可选 Sensor/）
4. 写 mount：simulationOwner 清空；TF；scale 无负分量
5. RG75.usda 仅追加 Pad=/Sensors=；不改默认选中
6. 若用户授权且需要 → 再改 Side / Physics（否则跳过）
7. PhysX 验收；确认其它 Pad 未被动
8. 删临时整爪目录（若在 RG75 树内临时建过）
```

`finger_support`：B/C 默认 `inherited`（detector）；type1 可 `invisible`。

---

## 5. Golden 索引

| 资产 | 路径 | 备注 |
|------|------|------|
| 入口 | `RG75.usda` | 只追加 variant |
| type1 / detector | `Pad/type1/` `Pad/detector_type1/` | B |
| heavy | `Pad/heavy_carry/` | A；Side 已有 sticky（历史） |
| load_type1 | `Pad/load_type1/` | A，无 Side |
| load_type2 | `Pad/load_type2/` | C，无 Side |
| scanner | `Sensor/scanner_type1/` | Sensors |
| Side | `Side/left\|right.usda` | **默认只读** |

---

## 6. Agent 规范

1. 先问诊模板，再改文件。  
2. Diff 自检：是否越出 `Pad/` `Sensor/` + 入口追加？是否动到其它 Pad / Side / 父机？  
3. 文件名不可信（`pad_right.usdc` 可能挂左指）。  
4. [reference.md](reference.md) · [examples.md](examples.md)  
5. MuJoCo/执行器细节见 `fasim-robot-mujoco-physics`（不替代本边界）。

## Additional resources

- [track-a-bake.md](track-a-bake.md)
- [track-b-usdc-mount.md](track-b-usdc-mount.md)
- [track-c-simplified-collision.md](track-c-simplified-collision.md)
- [reference.md](reference.md)
- [examples.md](examples.md)
