---
name: fasim-rg75-pad-convert-reference
---

# RG75 Pad Convert — Reference

## 术语

| 术语 | 含义 |
|------|------|
| **Pad** | 根 variant：换 tip；选项名=用途 |
| **side** | 关节 `left_`/`right_` 前缀；可 sticky 相机/`arm` |
| **arm**（nested） | **臂间** tip CAD；仅 heavy 式；Side sticky |
| **指间 L/R** | 同一夹爪左右指 tip；Track C 两 usdc；**不**靠 Side 互换 |
| **gripper_base** | 新底座（旧 flange） |

旧名 `left_left` = 臂×指；进 `left/`/`right/` 目录，勿进 Pad 下拉。

## 目录模板

### Track A type1-like

```
Pad/<name>/
  <name>.usda
  geometries.usd | instances.usda | materials.usda | Textures/
  # 或 pad_left.usda + pad_right.usda
```

### Track A heavy-like

```
Pad/<name>/
  <name>.usda          # nested arm
  left/ … pad_left.usda pad_right.usda
  right/ …
```

### Track B / C

```
Pad/<name>/
  <name>.usda          # 或 payload.usda
  pad.usdc             # Track C 另加 pad_right.usdc（镜像）
```

## Side sticky（默认只读；须用户授权才追加）

```usda
# Side/left.usda — prim 路径必须与该 Pad mount 一致（heavy: pad_left/pad_right）
over "gripper_base" {
  over "pad_left" ( variants = { string arm = "left" } ) { }
}
over "right_finger" {
  over "pad_right" ( variants = { string arm = "left" } ) { }
}
```

right Side → `arm = "right"`。  
**load_type1 / load_type2 / type1 / detector：不要加 Pad tip sticky。**

## mujoco collision overs（可选）

路径必须属于该 Pad；`delete` Physx* collision API，保留通用 Physics*。见 heavy 提交版。

```usda
rel physics:simulationOwner   # 空
prepend rel material:binding:physics = </RG75/Looks/PadMaterials>
```

## 踩坑（实战）

1. 勿默认 type1 mimic——先看 tip 挂 base 还是 finger。  
2. Transformer 常把固定 tip 挂错在 flange。  
3. 占位 finger 要 invisible。  
4. 优先 geometries bake；旧 component usdc 易错位。  
5. 保留 mesh 内 0.001 scale。  
6. **禁止 mount `scale=-1`**；不规则 → Track C 两 usdc。禁止 AI pad_mirror 主路径。  
7. 先比 L/R mesh hash，再决定要不要 `left/`/`right/` + Side。  
8. PhysX 验收；不 chase Newton tip。  
9. 用途名；`ptc`→`load_type1`。  
10. 禁止改 RG75↔TCP 救单 Pad。  
11. `variantSets`：`side` 在 `Pad` 前。  
12. Pad 下拉一个名字；臂差用 nested `arm`。  
13. W2/M6 只动 payload/variant。  
14. 清 `simulationOwner`。  
15. 指间包 **不要** 按 side 互换 tip（scanner 在 base 会「对翻」）。  
16. 视觉 mesh 与 Cube 勿双碰撞。  
17. 文件名 `pad_right` ≠ 挂右指；看旧 scale=-1 侧。  
18. 校验用 session layer；默认 `Pad=none`。  
19. instanceable tip 用 prototype 查 mesh。  
20. 验收后删临时整爪目录；robots 子模块单独 commit。

## 校验（可选）

session layer 设 `Pad`/`side`/`Sensors`，查 tip / scanner prim 存在与 visibility；Isaac 看碰撞与开合。
