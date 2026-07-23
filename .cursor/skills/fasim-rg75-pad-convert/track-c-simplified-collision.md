# Track C — 人工简化碰撞 tip USDC（cube colliders）

人工在 Isaac 里用 **Cube 简化碰撞**做好的 tip `*.usdc`，挂到 RG75 `Pad=<purpose>`。  
**禁止** AI mesh bake / `pad_mirror` / mount `scale=-1`。  
Golden：`payloads/Pad/load_type2/`（`pad.usdc` + `pad_right.usdc` + `load_type2.usda`）。

## 何时用

| 信号 | Track C |
|------|---------|
| 用户/领导已给带 `/root/collision/Cube*` 的 tip usdc | ✅ |
| 不规则 tip，不能靠旋转对齐左右 | ✅（两份 usdc 手动镜像） |
| 只要 Asset Transformer bake / 单 usdc+负 scale | ❌ → Track A / 旧 B |

## 人工 GUI 前置（人做，Agent 不替代）

1. Isaac 打开 **一份** tip：`pad.usdc`（defaultPrim=`/root`）。
2. 在 `/root` 下建 `collision`（Xform），`visibility=invisible`。
3. 在 `collision` 下 **Create → Mesh → Cube**，拖到贴合视觉 mesh；可多个（Cube_01…）。
4. 每个 Cube：Physics → Collision ON；Approximation = **`boundingCube`**（或 convexHull）。
5. Cube 可视可关（父 `collision` invisible 即可）；**视觉 mesh 不要开 Collision**。
6. Save `pad.usdc`。
7. **Save As** → `pad_right.usdc`；在副本里手动调 **mesh + cubes** 到另一指外形（可改 translate/orient；尽量避免长期负 scale）。
8. 把两文件丢给 Agent：`Pad/<purpose>/pad.usdc` + `pad_right.usdc`（或 `pad_left.usdc` + `pad_right.usdc`）。

## 为何左右 Stage 会「同时」出现 collision

两指若都 `payload = @./pad.usdc@`，改一份 usdc **两边一起变**——这是 USD reference，不是挂载写错。  
Track C **必须**左→`pad.usdc`、右→`pad_right.usdc`（两路径）。

## Agent 挂载清单

```
- [ ] 1. 确认 usdc：视觉 Mesh 无 Collision；/root/collision/Cube* 有 Collision
- [ ] 2. 用几何语义挂载，勿只看文件名：对比旧 mount 谁曾 scale=-1
      · 曾 scale=-1 的那一侧 → 挂镜像 usdc（load_type2：左指 → pad_right.usdc）
      · 无 scale 的一侧 → 挂 canonical（右指 → pad.usdc）
- [ ] 3. mount scale=(1,1,1)；保留源 rotate；禁止 mount 负 scale
- [ ] 4. 关视觉 mesh 碰撞；cubes 清 simulationOwner + PadMaterials
- [ ] 5. nested arm：若仅有「指间 L/R」两套（无独立臂 CAD）→ arm=left/right **内容相同**
      （scanner 在 gripper_base 上；勿按 side 互换 tip，否则一侧 pad↔scanner 对翻）
      仅当另有左臂/右臂专用 tip 包时才像 heavy 那样按 arm 换文件
- [ ] 6. Side sticky 仍挂 arm（与 heavy 兼容；Track C 指间包时 sticky 为 no-op）
- [ ] 7. 登记 Pad；PhysX 验左右臂上 scanner 相对 tip 一致
```

## 文件名陷阱（load_type2）

| 文件 | 几何 | 挂哪指 |
|------|------|--------|
| `pad.usdc` | canonical（旧共用 mesh） | **右指** |
| `pad_right.usdc` | ≈ `pad.usdc`×`scale(-1,1,1)`（Save As 手调镜像） | **左指** |

`pad_right` 是 Save As 文件名，**不是**「一定挂 right_finger」。以旧 PhysX 挂载 / 点云镜像关系为准。

## 为何 scanner「跑到对面」

`scanner_type1` 固定在 `gripper_base`，不随 `side` 改 TF。  
若按 side **互换**左右 tip usdc，夹爪坐标系里 pad 相对底座翻了，看起来就像 scanner 在正确位置的对面——其实是 **pad 挂反了**。右臂曾「碰巧正常」是因为错误的 swap 把镜像文件换到了左指上。

## 碰撞冲突（必查）

| 错误 | 现象 | 修复 |
|------|------|------|
| Mount 给视觉 mesh 写了 convexDecomposition | mesh+cube 双碰撞 | delete mesh 碰撞 API；`collisionEnabled=0` |
| 按文件名把 `pad_right` 挂右指 | 左臂 pad↔scanner 对翻 | 镜像文件挂**左指**；canonical 挂右指 |
| Mount `scale=(-1,…)` | Newton 炸 | 用第二份 usdc，scale=1 |

## 输入约定（用户丢给 Agent）

最少：

```
payloads/Pad/<purpose>/
  pad.usdc         # canonical（通常→右指）
  pad_right.usdc   # 镜像（通常→左指；以旧 scale=-1 侧为准）
```

可选说明：左右 mount `rotateZYX`（缺省对齐 load_type2：`(-90,0,0)` / `(-90,180,0)`）。
