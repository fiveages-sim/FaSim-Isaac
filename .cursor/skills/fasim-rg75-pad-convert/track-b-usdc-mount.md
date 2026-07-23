# Track B — 薄 USDC 挂载

现成 tip **`.usdc`**（或等价自包含 tip），用 mount USDA 挂到左右指；  
左右指优先靠 **旋转** 对齐，**禁止** mount `scale=-1`。  
Golden：`type1/`（`pad.usdc`）、`detector_type1/`（`payload.usda` + `pad.usdc`）。

## 何时用

- 已有 tip usdc，**没有**（或不需要）人工 Cube 碰撞树
- 形状允许 L/R 用同一 usdc + 不同 `rotate`/`orient`
- 碰撞：无 / mesh 近似 / 源 usdc 已带碰撞 API

## 何时改走 Track C

- 不规则，旋转对不齐，必须镜像  
- 用户/领导交付 `/root/collision/Cube*` 简化碰撞  

## 步骤

1. 确认问诊：purpose 名、TF 来源、碰撞策略、有无 Sensor。
2. 放入 `payloads/Pad/<purpose>/`（`pad.usdc` + `<purpose>.usda` 或 `payload.usda`）。
3. 左右 `payload=@./pad.usdc@`（或分文件但无负 scale）；`scale=(1,1,1)`。
4. `finger_support`：默认 **`inherited`**（对齐 detector；type1 可 invisible）。
5. 清 `simulationOwner`；按需绑 `PadMaterials`。
6. 登记 `Pad=<purpose>`；**通常不改 Side**。
7. 可选 `Sensors=<name>`（scanner 挂 `gripper_base`，TF 常与臂无关）。
8. PhysX 验收；session layer 切 variant，勿写进金色默认。

## 注意

- 源 tip 内部偶发负 scale：PhysX 可保留在 **usdc 内部**；不要在 mount 再叠 `scale=-1`。
- GUI Save tip 时避免把 `Render`/`PhysicsScene` 写进 tip（detector payload 有脏数据前例）。
- 两指同 `@./pad.usdc@` 时改 usdc 两边一起变——若以后要分镜像，改走 Track C 两文件。
