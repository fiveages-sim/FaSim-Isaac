---
name: fasim-usd-bake-scale
description: >-
  Bake non-unit xformOp:scale on FaSim USD/USDC assets into mesh points (and child
  translates) while preserving world size, then set scale to (1,1,1). Use when
  cleaning CAD inch→meter 0.0254 scales, shelf/static env assets with parent 0.5
  plus unit-cube collision scales, or when the user asks to bake scale / freeze
  scale / keep size but scale=1 on .usd/.usda/.usdc.
---

# FaSim USD Bake Scale

保持**当前世界尺寸**，把 `xformOp:scale ≠ (1,1,1)` 烘焙进几何（及子节点 translate），最后把 scale 写成 `(1,1,1)`。

## 何时用

- 环境/物体资产（`fiveages_env/static/…`、`fa-project-usd/**/objects/*.usdc`）带非单位 scale
- CAD 英寸导出残留：`≈0.0254` 均匀 scale
- 父节点均匀缩放 + 子碰撞体「单位立方体 × 非均匀 scale」（如 `shelf8.usd`）
- 用户说：烘焙 scale / freeze scale / 保持大小但 scale=1

**不要**用本 skill 处理：RG75 mount `scale=-1`（见 `fasim-rg75-pad-convert`）、MuJoCo Physics bake（见 `fasim-robot-mujoco-physics`）。

## 工具

优先 Isaac Sim 自带 pxr（系统 python 通常无 `pxr`）：

```bash
USD_LIBS="$HOME/isaacsim/extscache/omni.usd.libs-"*".lx64.r.cp312"
# 取实际目录名；示例：
USD_LIBS=$(echo "$HOME"/isaacsim/extscache/omni.usd.libs-*-lx64.r.cp312 | awk '{print $1}')
export PYTHONPATH="$USD_LIBS${PYTHONPATH:+:$PYTHONPATH}"
export LD_LIBRARY_PATH="$USD_LIBS/bin${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}"
"$HOME/isaacsim/kit/python/bin/python3" .cursor/skills/fasim-usd-bake-scale/scripts/bake_scale.py <asset.usd>
```

脚本：[`scripts/bake_scale.py`](scripts/bake_scale.py)（`--dry-run` 只报告；默认原地保存）。

## 算法（自上而下）

对每个 `UsdGeom.Xformable`，xform 顺序为 `translate → rotate/orient → scale`（`T * R * S`）：

1. 读本地 `xformOp:scale = (sx,sy,sz)`；若已是单位则跳过并递归子节点。
2. **本 prim 是 Mesh**：`points *= S`；非均匀时对 `normals` 做 `diag(1/sx,1/sy,1/sz)` 再归一化；更新 `extent`。
3. **每个直接子节点**：
   - 有 translate → `(tx,ty,tz) *= (sx,sy,sz)`（分量乘）
   - 有 scale → 子 scale 再乘父 S（留给子节点后续烘焙）
   - 子 Mesh **且无** scale op → 立刻把父 S 烘焙进该 mesh 的 points/extent
4. 将本 prim 的 scale 设为 `(1,1,1)`（保持 `Vec3f`/`Vec3d` 类型）。
5. 递归子节点。

均匀父 scale（如 `0.5` 或 `0.0254`）下，子旋转无需共轭修正；当前资产均属此类或轴对齐非均匀。

## 验收

1. 烘焙前后 **world bbox size** 相对误差 ≪ `1e-6`（理想为 0）。
2. Traverse 后无 `|scale_i - 1| > 1e-6` 的 `xformOp:scale`。
3. 抽查碰撞体：world 尺寸 ≈ 原「父 scale × 子 scale × 单位体」；translate 已吸收父 scale。

## 已验证样例

| 资产 | 典型 scale |
|------|------------|
| `environment/fiveages_env/static/shelf/shelf8/shelf8.usd` | 根 `0.5`；`collisions/*` 单位立方体非均匀 scale |
| `environment/fa-project-usd/Kunhua/.../objects/task{1,2}/*.usdc` | 叶节点 `≈0.0254`（英寸→米） |

## 注意

- 大 mesh（如 shelf8 视觉 mesh ~1M points）用脚本处理即可，勿手改 crate。
- 场景 `.usda` 里仅浮点噪声（`1.0000001`）可直接改成 `(1,1,1)`，不必烘焙几何。
- 修改后若场景引用该资产，无需改引用路径；仅资产内部意见变化。
