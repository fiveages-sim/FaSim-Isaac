# Track C — 人工简化碰撞 tip USDC（Cube）

人在 Isaac 用 **Cube** 做好简化碰撞的 tip usdc，再由 Agent 挂到 `Pad=<purpose>`。  
**禁止** AI `pad_mirror` / mount `scale=-1`。  
Golden：`payloads/Pad/load_type2/`（`pad.usdc` + `pad_right.usdc` + `load_type2.usda`）。

## 何时用

| 信号 | |
|------|--|
| tip 内已有 `/root/collision/Cube*`（`boundingCube` 等） | ✅ |
| 不规则，不能靠旋转对齐左右指 | ✅ 两份手调镜像 usdc |
| Transformer bake / 单 usdc+旋转即可 | ❌ → A / B |

## 人工 GUI 前置（人做，Agent 不替代）

1. 单独打开 tip usdc（defaultPrim `/root`），勿开整爪以免脏写。
2. `/root` 下建 `collision`（Xform），`visibility=invisible`。
3. 内建多个 Cube，贴合视觉 mesh。
4. 每 Cube：Collision ON；Approximation = **`boundingCube`**（或 convexHull）。
5. **视觉 Mesh 不要开 Collision**。
6. Save 第一份；**Save As** 第二份，手调 mesh+cubes 到另一指。
7. 交给 Agent：两文件 + purpose 名 + 已知 rotate（若有）。

## 为何 Stage 左右同时出现 collision

两指若都 `payload=@./同一.usdc@`，改一份两边变。Track C **必须两个路径**。

## 挂载语义（勿只看文件名）

以旧 PhysX / 点云为准：

| 几何 | load_type2 文件名 | 挂哪指 |
|------|-------------------|--------|
| canonical（旧无 scale=-1 侧） | `pad.usdc` | **右指** |
| X-mirror（旧 `scale=(-1,1,1)` 侧） | `pad_right.usdc` | **左指** |

`pad_right` = Save As 名，**≠** 一定挂 `right_finger`。

load_type2 mount：

- 左：`pad_right.usdc` + `rotateZYX=(-90,0,0)` + `scale=(1,1,1)`
- 右：`pad.usdc` + `rotateZYX=(-90,180,0)` + `scale=(1,1,1)`

## Side：默认不改

指间 L/R 固定在夹爪坐标系 → **与 load_type1 一样不改 Side**。  
`scanner` 在 `gripper_base`：若按 side **互换** tip，会出现「scanner 跑到对面」（其实是 pad 挂反）。  
仅当另有**左臂/右臂专用 tip CAD**（heavy）才 nested `arm` + Side sticky。

## Agent 清单

```
- [ ] 视觉 Mesh collisionEnabled=0；仅 Cube 碰撞
- [ ] 按几何语义挂左右指；mount 无负 scale
- [ ] 清 simulationOwner；Cube 绑 PadMaterials
- [ ] 不改 Side（除非确认臂间 CAD）
- [ ] 登记 Pad；PhysX 验开合 + 左右臂 scanner↔tip
```

## 碰撞冲突

| 错误 | 修复 |
|------|------|
| mesh + cube 双碰撞 | 关 mesh 碰撞 |
| 按文件名把镜像挂错指 | 对齐旧 scale=-1 侧 |
| mount `scale=-1` | 用第二份 usdc |
