# 诊断脚本（diagnostics）

这些脚本是**调试期的一次性工具**，不是平台功能的一部分。
保留它们是因为里面的判据和排查思路有参考价值（很多结论已写进
`docs/DEBUG_LOG.md`），但不参与日常采集/导出流程。

## 分类

| 脚本 | 用途 | 对应 DEBUG_LOG 章节 |
|---|---|---|
| `probe_camera_params.py` | 相机内参/外参字段探针（升级 Isaac 后复核） | #3 |
| `probe_cameras.py` / `probe_camera_orientation.py` | 相机 prim / 朝向排查 | #3 |
| `check_camera_live.py` | 实时相机是否出图 | #9 |
| `ablate_cameras.py` | 单相机消融（判断哪路图像在拖后腿） | — |
| `compare_camera_inputs.py` | 数据集图 vs 实时图 像素对比 | 图像管线 |
| `compare_policy_inputs.py` | 同一 state 下数据集图/实时图的策略输出差 | 图像管线 |
| `test_head_conversion.py` | head 相机 16:9↔4:3 转换方案对比 | #3 |
| `probe_ee_tracking.py` | 运控末端跟随探针（闸门/阶跃/步长扫描/轨迹回放） | #16 |
| `probe_infer_chain.py` | 推理链路四阶段探查（hold/ocs2/lock/阶跃） | #17 |
| `probe_infer_run.py` | 推理运行期采样（底盘/目标/夹爪/拼图） | #17 #19 |
| `check_chunk_reach.py` | ACT chunk 累积位移检查 | #16 |
| `verify_action_queue.py` | 验证 n_action_steps 覆盖是否生效 | #19 |
| `prune_idle_prefix.py` | 裁掉开头静止段（**已被 `prep_act_dataset.py` 取代**） | — |

## 正式工具（不在本目录）
- `scripts/prep_act_dataset.py` —— 训练前数据整理（删坏run/裁静止/重排序号/pack）
- `scripts/rollout_openloop.py` —— 开环策略回放（判断轨迹正确性）
- `scripts/cli.py` / `sdg.sh` —— 主入口
