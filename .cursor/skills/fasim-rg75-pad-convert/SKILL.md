---
name: fasim-rg75-pad-convert
description: >-
  Converts legacy Jodell RG75-with-pad gripper USD assets into modern Pad
  payloads on robots/grippers/Jodell/RG75. Tracks: A bake from Asset Transformer;
  B thin USDC mount; C human cube-collider tip USDCs (pad.usdc + pad_right.usdc,
  Side sticky swap, no scale=-1). Use for heavy_carry, type1, load_type1,
  load_type2, detector pads, simplified Cube collision tips, Side sticky arm.
---

# FaSim RG75 Pad Convert

把 **Asset Transformer 导出的旧 RG75+夹指 USDA** 拆成 Pad payload，挂到参考底座
`robots/grippers/Jodell/RG75/` 的 `Pad` variant 上。底座/关节已在 RG75；新资产只贡献夹指。

**Golden references（先读再改）：**
- 根与 variants：`robots/grippers/Jodell/RG75/RG75.usda`
- 对称双指同 mesh：`payloads/Pad/type1/`（`type1.usda` + `pad.usdc`）
- 非对称 + 左右 CAD + Side sticky：`payloads/Pad/heavy_carry/`
- type1-like bake：`payloads/Pad/load_type1/`
- **USDC 薄挂载**：`payloads/Pad/detector_type1/`
- **Track C 简化碰撞**：`payloads/Pad/load_type2/`（`pad.usdc` + `pad_right.usdc` + cube colliders）
- Side 关节前缀：`payloads/Side/left.usda` | `right.usda`
- Sensor 扫码器例：`payloads/Sensor/scanner_type1/`

## 三轨起点

| Track | 何时用 | 产物 |
|-------|--------|------|
| **A — bake** | 有左右整爪 Asset Transformer USDA | geometries + instances，体积大，少负 scale |
| **B — USDC** | 现成 tip usdc + 已知 TF；可用旋转对齐 | 单/`N` 个 tip usdc + 薄 mount；**禁止** mount `scale=-1` |
| **C — 简化碰撞 USDC** | 人已在 tip usdc 里摆好 `/root/collision/Cube*`；不规则只能镜像 | `pad.usdc` + `pad_right.usdc`；mount 只挂碰撞 cube；`arm`+Side 互换文件 |

**拿到已简化碰撞的 tip usdc → 走 Track C**，细则：[track-c-simplified-collision.md](track-c-simplified-collision.md)。  
`load_type2` 是 Track C golden（已弃用 AI `pad_mirror`）。

Track B/C 视觉：`finger_support` 默认 **`inherited`**（对齐 `detector_type1`）；勿照抄 type1 的 `invisible`。

**验收引擎：PhysX（主路径）。**  
本 skill **不负责** Newton/MJWarp tip 调参。

## Pad variant 命名（按用途 / 场景）

最终验收时，`Pad` 下拉项必须是 **用途/场景名**，不要用厂商内部代号或临时工程名单独当 variant 名。

| 规则 | 说明 | 例子 |
|------|------|------|
| 用途优先 | snake_case，读得出「干什么」 | `heavy_carry`、`load_type1` |
| 文件夹 = variant 名 | `payloads/Pad/<name>/` + 入口 `<name>.usda` | `Pad/load_type1/load_type1.usda` |
| 禁止裸代号 | 旧 CAD/项目名可写在 doc / 源路径注释里 | ❌ `ptc` → ✅ `load_type1`（doc 注明 legacy PTC） |
| 已有惯例保留 | 已上线的 `type1`、`heavy_carry` 不改名 | `type1` 继续用 |

转换前先和用户确认用途标签；不确定时问一句再登记 variant。

**硬约束：**
- 不破坏 `type1`；尽量只 **新增** `Pad/<purpose_name>/` + 在 `RG75.usda` 登记同名 variant +（必要时）Side sticky；heavy 可按需在共享 `Physics/mujoco.usda` 留 collision overs（与已提交版一致）。
- **不要**用整爪替换底座；只抽 pad。
- 双臂 L/R：**优先独立 CAD 或合法旋转**；勿用 mount `scale=-1`。不规则 tip → **Track C** 两份 usdc（人工 cube 碰撞），见 [track-c-simplified-collision.md](track-c-simplified-collision.md)。
- 不要为某一款 pad 改 RG75 相对 TCP 的全局朝向（会搞坏其他 Pad）。
- 双臂父机（W2/M6）：优先只改 EE/Pad **payload 或 variant sticky**；勿改父机核心代码/非 payload 层。

细节模板与踩坑见 [reference.md](reference.md)；实录见 [examples.md](examples.md)。

## Checklist

```
- [ ] 0. 起点：Asset Transformer 原始 USDA（可读）已就绪；对照 RG75 结构
- [ ] 1. 审计：命名、挂载父节点、开合运动学、是否分左右 CAD（先比 L/R mesh hash）
- [ ] 1b. 确认 Pad **用途名**（heavy_carry / load_type1 / …），文件夹与 variant 同名
- [ ] 2. 分类 kinematics + L/R 策略，选定 type1-like / heavy-like / 自定义
- [ ] 3. 从 geometries/instances 抽出 pad 包（优先 bake，勿盲抄旧 component usdc）
- [ ] 4. 写 mount USDA：挂到正确 prim；清 simulationOwner；隐藏冲突视觉
- [ ] 5. 登记 Pad=<purpose_name>；若需 L/R CAD → nested arm + Side sticky + variantSets 顺序
- [ ] 6. PhysX 可视化校验（开合、左右臂、ROS2）；删临时整爪文件夹；提交
```

## 0. 起点与对照表

| 旧 Asset Transformer | 新 RG75 |
|----------------------|---------|
| 根常为 `Jodell_RG75` | 根 `RG75` |
| `flange` | `gripper_base`（等价） |
| `left_finger` / `right_finger` | 同名 |
| 整爪含底座+关节+pad | **只保留 pad**；底座/关节用参考 RG75 |
| 旧 `left_left` 等命名 | =「臂侧」×「指侧」；映射到 mount 路径，勿照搬目录名 |

`side` **不是**左右夹指镜像，而是给双臂控制器的 **关节改名**：

| `side` | 主动关节 | mimic 从动 |
|--------|----------|------------|
| `default` | `gripper_joint` | `right_finger_joint` |
| `left` | `left_gripper_joint` | `left_right_finger_joint` |
| `right` | `right_gripper_joint` | `right_right_finger_joint` |

几何仍是同一套 `left_finger` / `right_finger`。换夹指用 **`Pad`**。

## 1. 审计源资产

打开 Transformer 根 `.usda` + `payloads/base.usda` / `instances.usda` / `geometries.usd`，记录：

1. **Pad mesh 路径**（哪些 prim 是夹指 tip，哪些是底座/连杆）。
2. **父节点**：tip 挂在 `flange`/`gripper_base` 还是 `*_finger`？（Transformer 常把固定 tip 错挂在 flange 下——见踩坑。）
3. **开合**：双指都动（type1）还是一侧固定一侧动（heavy）？占位 finger 是否应 invisible？
4. **左右 CAD**：同一 mesh 两朝向够用，还是 Left/Right 臂各有一套 CAD（禁止 **双臂** scale=-1）？
5. **单位**：mesh 上常有 `scale≈0.001`（mm→m）；保留在 pad 内部 xform，不要丢。

**必做：先比 Left vs Right（省大量无效工作）**

```bash
# 1) 逐 Mesh 点云 hash（不要只比 geometries.usd 文件 md5——元数据常不同）
# 2) 抽出两侧 base.usda 里每个 pad_* 块，diff 是否字节级相同
```

| 结果 | 结论 |
|------|------|
| 全部 pad mesh hash 相同 + pad 块相同 | **只 bake 一侧**；`side` 只管关节名；**不要** nested `arm` |
| mesh 或 pad 块不同 | 再考虑 `left/`/`right/` + Side sticky |

校验组合时用 **session layer** 设 `Pad=<name>`，**禁止**把选择写进金色 `RG75.usda`（默认保持 `Pad=none`）。

## 2. 分类 → 文件夹形态

### A. type1-like（双指都随 finger 动 / mimic）

```
payloads/Pad/<pad_name>/
  <pad_name>.usda          # mount：left_finger/pad + right_finger/pad
  geometries.usd           # 仅 tip mesh subset
  instances.usda + materials.usda (+ Textures/ 若需要)
  pad.usdc                 # 若左右 tip 内部 xform 相同：可单文件（参考 type1）
  # 或当左右 tip 子件 transform 不同（常见）：
  pad_left.usda + pad_right.usda   # 共享同一 geometries，无 nested arm
```

两指都挂 tip；容器 `rotateZYX` 可不同；隐藏 `finger_support`。  
**注意：**「左右臂 CAD 相同」≠「左指/右指 tip 包相同」——右指软垫常在源资产里带不同子件 TF（含偶发 `scale=(-1,1,1)`）。  
PhysX 路径可保留源 TF；勿为双臂镜像新造负 scale。

### B. heavy-like（固定 tip + 运动 tip；可有左右 CAD）

```
payloads/Pad/<pad_name>/
  <pad_name>.usda          # 或 heavy.usda：mount + nested variantSet "arm"
  left/                    # 左臂 CAD（geometries.usd, instances, materials, pad_*.usda）
  right/                   # 右臂 CAD（若需要）
```

典型运动学：
- **真·固定 tip** → 挂在 `gripper_base`（随底座不动）。
- **运动 tip** → 挂在 `right_finger`（随开合）。
- 源里的 `left_finger` 可能只是占位 → `visibility = invisible`。

### C. 仅一侧有专用 CAD / 另一侧可旋转复用

优先：**嵌套 `arm=left|right` + Side sticky**（Pad 下拉只有一个选项）。  
不要做成 `Pad=foo_left` / `foo_right` 两个入口（双臂 UX 差）。

## 3. 抽出 pad 几何（推荐 bake）

1. 从 Transformer 的 **`geometries.usd` + `instances.usda` + `materials.usda`（+ Textures）** 复制出 **仅 pad 相关** 的 subset。
2. 每个 tip 一个入口：`pad_left.usda` / `pad_right.usda`，`defaultPrim = "root"`，内部 `mesh` `references = @./instances.usda@</Instances/…>`，保留 mesh 本地旋转/0.001 scale。
3. **不要**优先复用旧 `components/*.usdc`：与合成层朝向不一致时，对齐会算出离谱位移；heavy 实测 usdc 路线导致左臂错位，已回退。
4. 临时整爪目录（如 `RG75_Heavy_Left`）bake 进 `Pad/<name>/` 并验证后 **删除**，保持 `Jodell/` 整洁。

## 4. 写 mount USDA

Mount 文件 `defaultPrim = "RG75"`，用 `over`/`def` 接到参考结构：

- 清碰撞归属：`rel physics:simulationOwner`（空 rel，对齐 type1）。
- 绑定摩擦可选：`Looks/PadMaterials` + `material:binding:physics`。
- 隐藏冲突视觉（占位 finger、`finger_support` 等）。
- Mount TF：以 **源资产 rest 位姿** 对齐到参考 finger/base；验证开合后再冻结。不确定时 **保持已验证 TF**，勿为「看起来转了 180」盲目改底座。

最小 type1 挂载形态（示意）：

```usda
def "RG75" {
  over "left_finger" {
    rel physics:simulationOwner
    over "visuals" { over "finger_support" { token visibility = "invisible" } }
    def "pad" ( prepend payload = @./pad.usdc@ ) {
      double3 xformOp:translate = (0, 0, 0)
      double3 xformOp:rotateZYX = (...)
      uniform token[] xformOpOrder = ["xformOp:translate", "xformOp:rotateZYX", "xformOp:scale"]
      # clear simulationOwner on collision meshes
    }
  }
  over "right_finger" { /* 对称，orient 可不同 */ }
}
```

heavy + 左右 CAD：在 `pad_left`/`pad_right` 上嵌套 `variantSet "arm"`，payload 指向 `./left|right/pad_*.usda`，各写自己的 mount TF。完整样例见 [examples.md](examples.md)。

## 5. 登记 variant + Side sticky

在 `RG75.usda` 的 `variantSet "Pad"` 增加一项：

```usda
"<pad_name>" (
    prepend payload = @./payloads/Pad/<pad_name>/<mount>.usda@
) { }
```

**variantSets 顺序（必守）：**

```usda
prepend variantSets = ["Physics", "flange", "side", "Pad", "Sensors"]
```

`side` 必须在 `Pad` **之前**，Side 里对 nested `arm` / Orbbec `mount` 的 sticky 才能压过 Pad 默认值。

若 Pad 有 nested `arm`，在 `Side/left.usda` / `right.usda` 增加 sticky（无 Pad 时 no-op）：

```usda
over "gripper_base" {
  over "pad_left" ( variants = { string arm = "left" } ) { }  # right Side → arm = "right"
}
over "right_finger" {
  over "pad_right" ( variants = { string arm = "left" } ) { }
}
```

W2/M6：臂的 EE 挂载已 sticky `side=left|right` 时，选 `Pad=<pad_name>` 即可自动选对 CAD，无需再选 `heavy_carry_left`。

## 6. 物理（PhysX 主路径）

对齐 type1：mount 层清 `simulationOwner`；tip `instances` 写 `PhysicsCollisionAPI` + 合适 `approximation`（heavy 已提交版用 `convexDecomposition`，PhysX 正常）。

可选：在共享 `Physics/mujoco.usda` 对 heavy 路径留 collision overs（与 `ee8ed8d` 一致）——**不**为 Newton 去改成 `convexHull` 或删 overs。  
底座 Side/`MjcActuator` 见 `fasim-robot-mujoco-physics`；**Pad 转换本身不 chase Newton tip 问题。**

## 7. 验收与收尾

- [ ] `Pad=none|type1|heavy_carry|<new>` 切换无组合错误、无丢材质。
- [ ] **PhysX** 下开合正确；`side=left|right` 关节前缀与 arm sticky 正确。
- [ ] 双臂挂载视觉 OK；勿为单 pad 改 RG75↔TCP。
- [ ] 删除临时 Transformer 整爪目录；commit。

## 决策速查

| 现象 | 做法 |
|------|------|
| 双指同 mesh、都随指动 | type1-like |
| 一 tip 固定在 base、一 tip 在 finger | heavy-like；invisible 占位 finger |
| 左右臂 mesh 不同且不能镜像 | `arm` nested + Side sticky |
| 对齐错位 / 位移离谱 | 弃旧 usdc，从 geometries bake |
| 右臂整体差 Rz180，但其他 Pad 正常 | 勿改 RG75↔TCP；只调该 pad 或 EE 层 |
| Pad 下拉出现 `_left/_right` | 合并为一个 Pad + sticky |
| Newton 下 tip/开合异常 | **不在本 skill 范围**；PhysX 继续用已提交 Pad |

## Additional resources

- [reference.md](reference.md) — 踩坑清单、命名、模板
- [examples.md](examples.md) — heavy_carry 转换实录
- Skill `fasim-robot-mujoco-physics` — 夹爪/Side MuJoCo 与父机挂载
