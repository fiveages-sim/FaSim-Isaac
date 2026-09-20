---
name: fasim-rg75-pad-convert-examples
---

# RG75 Pad Convert — Examples

新任务先走 SKILL **§0 问诊**，再对号入座。此处只记结论与禁区。

---

## heavy_carry — Track A + Side

**输入：** Transformer `RG75_Heavy_Left/Right`。  
**输出：** `Pad/heavy_carry/` + Side sticky `arm`。

| 项 | 结论 |
|----|------|
| 运动学 | 固定 tip@base；运动 tip@right_finger；left_finger 占位 invisible |
| 臂 CAD | 不同 → `left/`/`right/` + nested `arm` |
| Side | **要改** sticky |
| 失败过 | 旧 usdc 对齐；`Pad=*_left/right`；改 TCP；W2 非 payload |

---

## load_type1 — Track A，无 Side

**输入：** PTC Left/Right；mesh+pad 块相同。  
**输出：** 单侧 bake + `pad_left`/`pad_right` tip 包；**不改 Side**。

---

## detector_type1 / type1 — Track B

单 `pad.usdc`；左右靠旋转；**不改 Side**。  
`finger_support`：detector=`inherited`；type1 可 invisible。

---

## load_type2 — Track C，无 Side

**输入：** 人做好的 `pad.usdc` + `pad_right.usdc`（Cube 碰撞）。  
**挂载：** 左=`pad_right.usdc`（镜像）；右=`pad.usdc`；无负 scale；mesh 无碰撞。  
**Sensor：** `scanner_type1`@base；与 tip 相对位姿不随 side 变。  
**禁止：** AI pad_mirror；按 side 互换 tip；无必要改 Side。

---

## 下一款怎么开

1. SKILL 问诊表填完 → 宣布 Track。  
2. 打开对应 `track-*.md` + 本文件最近似例。  
3. Diff 检查 Side / RG75 根 / 父机边界。
