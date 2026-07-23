# Track A — Asset Transformer bake

从 **Asset Transformer 导出的左右整爪 USDA** 抽出 tip，bake 到
`payloads/Pad/<purpose>/`。  
Golden：`heavy_carry/`（非对称 + 臂 CAD）、`load_type1/`（对称臂 + 双 tip 包）。

## 何时用

- 用户有可读的 Transformer 根 `.usda` + `geometries` / `instances` / `materials`
- 还没有「已带 Cube 碰撞的 tip usdc」交付物

## 子形态

| 形态 | 信号 | 目录 | Side |
|------|------|------|------|
| **type1-like** | 两 tip 都在 `*_finger`；开合 mimic | `<name>.usda` + geometries 或 `pad_left/right.usda` | **不改** |
| **heavy-like** | 一 tip 固定 base、一 tip 在 `right_finger`；可有臂 CAD | `left/` `right/` + nested `arm` | 默认不改；要 sticky 须授权 |

### 必做：先比 Left vs Right

比 **Mesh points/indices hash** + pad 文本块（勿只比文件 md5）。

| 结果 | 做法 |
|------|------|
| 全相同 | 只 bake 一侧；无 nested `arm`；不改 Side |
| 不同 | `left/`+`right/` + nested `arm`；**Side sticky 须用户授权** |

## 步骤

1. 映射：`Jodell_RG75`→`RG75`，`flange`→`gripper_base`。
2. 定挂点：固定 tip→`gripper_base`；运动 tip→对应 finger；占位 finger→`invisible`。
3. 从 geometries/instances/materials **subset bake**（优先）；慎用旧 `components/*.usdc`（heavy 曾错位）。
4. 保留 mesh 内 `scale≈0.001` / 本地旋转。
5. 写 mount：清 `simulationOwner`；可选 `PadMaterials`。
7. `RG75.usda` **仅追加** `Pad=<purpose>`（不改默认选中）。
8. Side sticky / mujoco overs：**默认跳过**；仅当用户明确授权「可以改 Side / Physics」且确有臂间 CAD 时再动。
9. PhysX 验收；删临时 `RG75_*_Left/Right`（勿留在仓库）。

## 碰撞

- tip instances：`PhysicsCollisionAPI` + 合适 approximation（heavy：`convexDecomposition`）。
- 可选共享 `Physics/mujoco.usda` overs（路径必须存在于该 Pad）。
- **不为 Newton** 改成单 `convexHull` 或删 overs。

## 与 Track B/C

bake 完成后若用户改用 Cube 碰撞交付，下一轮按 **Track C 重挂**，不要混用 AI mirror。
