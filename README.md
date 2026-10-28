# FaSim-Isaac

FaSim-Isaac 是第五纪仿真平台的 Isaac Sim 资产快速部署仓库，旨在通过一键脚本将所有仿真所需的机器人 USD 模型与场景环境资产拉取到本地，并配置 ROS2 工作空间，方便用户快速搭建 Isaac Sim 仿真环境。

## 快速开始

### 1. 克隆主仓库

```bash
git clone git@github.com:fiveages-sim/FaSim-Isaac.git
cd FaSim-Isaac
```

### 2. 运行初始化脚本

```bash
./init.sh
```

脚本运行后首先选择操作类型：

#### 操作 1：初始化仓库（拉取子模块并切换到目标分支）

进入后勾选要初始化的子模块（列表来自顶层 `.gitmodules` 与 `submodules_visibility.conf`）：

- **public** 项默认勾选；**private** 项默认不勾选（需要对应仓库访问权限）
- 终端：↑/↓ 移动、空格勾选、`a` 全选、`n` 仅 public、Enter 确认、`q` 取消
- 非交互终端（管道/CI）：回车=仅 public，`a`=全选，或输入序号在 public 之外额外勾选
- 勾选嵌套子模块时会自动勾上其顶层父模块（例如 `robots`）
- 多次运行是**追加**：只 `update --init` 勾选项，不会卸载已有子模块

脚本会自动完成子模块初始化、分支切换并更新到最新提交。

#### 操作 2：更新子模块

不进入勾选菜单。已检出的子模块（含以前勾过的 private）会更新；尚未初始化的 public 会补拉；尚未初始化的 private 跳过。嵌套 public 会带上其顶层父模块。

默认保留当前分支，避免调试分支被切走。当前是 detached HEAD 时，改用 `.gitmodules` 的 `branch`（缺省 `main`）。本地改动会先暂存，落后远程时用 merge，冲突会列出文件。

```bash
./init.sh --update                      # 或 -u，保留当前分支
./init.sh --update --default-branch     # 或再加 -d，切换到 .gitmodules 分支
```

交互菜单第 2 项会再问一次是否切换到默认分支。主仓库本身不会 `git pull`。

#### 操作 3：加载 ArtVIP 场景

用 Isaac 自带的 Python 运行 `tools/artvip_kitchen.py download`，把官方数据下到 `environment/ArtVIP/`。场景组件在 `environment/artvip-usd`，不再转换出 `ArtVIP-fasim`。

#### 操作 4：配置 Isaac ROS2 Jazzy Workspace

可选 Isaac Sim 版本（菜单会查询 GitHub 最新 3 个稳定 tag，失败则回退 `config/fa_sim.conf`），下载对应 ROS workspaces，提取 `jazzy_ws` 到 `isaac_jazzy_ws/`，安装 rosdep / colcon 依赖，构建工作空间并写入 `~/.bashrc`。也可通过环境变量指定：`ISAAC_SIM_VERSION=6.0.1 ./init.sh`

#### 操作 5：安装 FaSim 扩展并启用脚本静默

只改本机 Isaac 安装目录（默认 `${HOME}/isaacsim`，可用 `ISAACSIM_DIR` 覆盖），不拉子模块、不构建 ROS 工作空间。重装 Isaac 后需要再选一次。正在运行的 Isaac 要重启后才生效。

- **舵轮控制节点**：`exts/fasim.steering.nodes` → `extsUser/`，写入 full / Newton / streaming kit。Linkhou / Taku 用 `SwerveController`，避免 ScriptNode 关掉 Action Graph。
- **ROS2 Control 节点**：`exts/fasim.ros2.nodes` → `extsUser/`，同上 kit。提供 `ROS2ControlLaunch`（in-process ControllerManager / emit_control，G1D·G1·W2 等）。
- **ROS / pink 策略**：`./run.sh` 设 `FASIM_FULL_HOST_ROS=1`——保留完整 `/opt/ros` LD+AMENT（workspace 在前），只清 PYTHONPATH，并禁用 `isaacsim.robot_motion.pink*`（与 apt pinocchio/eigenpy 冲突）。原版 `isaac-sim.sh` 仍走强隔离（剥掉 `/opt/ros` LD），好让 pink 示例能开。
- **脚本静默**：`opt_in = true` 与 `ignoreWarningDialog = true`（授权残留 ScriptNode、跳过 ArtVIP 行为脚本弹窗）。

### 3. 启动 Isaac Sim

```bash
./run.sh              # 普通 GUI
./run_headless.sh     # Headless Streaming
```

启动时先展示已保存的默认配置，再选：

1. **使用已选默认配置**（直接启动）
2. **重新选择**：物理引擎 PhysX/Newton、CPU performance、ROS2 工作空间（及 headless 实例号）

重新选择后可写入 `config/fa_sim.local.conf` 作为下次默认。

非交互示例：

```bash
./run.sh -y --physics physx --cpu auto --ros-ws ~/ros2_ws
./run_headless.sh -y --physics newton --instance 1   # → isaac-sim.newton.streaming.sh
./run_headless.sh -y --physics physx --instance 0    # → isaac-sim.streaming.sh
```

另：若 `RMW_IMPLEMENTATION=rmw_zenoh_cpp`，自动启动 Zenoh router，退出时清理。

### 配置说明

与 `fa_w2_ws` 类似，脚本共用 bash 配置文件：

| 文件 | 说明 |
|------|------|
| `config/fa_sim.conf` | 仓库默认配置（提交 git）：Isaac 路径、版本列表、apt 依赖等 |
| `config/fa_sim.local.conf` | 本机覆盖（gitignore）：交互保存或手改 |

```bash
cp config/fa_sim.local.template.conf config/fa_sim.local.conf
# 编辑 ISAACSIM_DIR / RUN_PHYSICS_BACKEND / RUN_ROS2_WS 等
```

---

## 文件结构

```
FaSim-Isaac/
├── init.sh                       # 一键初始化；选项 5 安装舵轮 + ROS2 Control 扩展
├── run.sh                        # Isaac Sim 普通 GUI 启动（交互选配置）
├── run_headless.sh               # Headless Streaming 入口
├── exts/fasim.steering.nodes/    # 舵轮控制 OmniGraph 节点
├── exts/fasim.ros2.nodes/        # ROS2ControlLaunch / emit_control
├── tools/artvip_kitchen.py       # 下载官方 ArtVIP
├── config/
│   ├── fa_sim.conf               # 共用配置（init / run）
│   └── fa_sim.local.template.conf

├── submodules_visibility.conf    # 嵌套子模块 public / private 配置
├── robots/                       # 机器人 USD 资产（Git 子模块）
├── environment/
│   ├── fiveages_env/             # 场景 USD 资产（Git 子模块）
│   ├── fa-project-usd/           # Fa Project USD 资产（Git 子模块，private）
│   ├── ArtVIP/                   # 原版下载（gitignore）
│   └── artvip-usd/               # 场景组件（payload 官方 ArtVIP）
└── isaac_jazzy_ws/               # ROS2 Jazzy 工作空间（由 init.sh 操作 4 生成）
```

`robots` 内按类型分子目录：人形（humanoid）、机械臂（manipulators）、夹爪（grippers）、灵巧手（dexhands）、传感器（sensors）、支架（stands）等；`environment` 下为场景与项目相关 USD。具体机型与文件名可直接在对应目录中查看。

---

## ArtVIP 场景

原版数据不入库。`./init.sh` 选 **3** 只下载官方数据。场景组件在 `environment/artvip-usd`，机器人 `env/artvip` 直接 payload `scenes/*.usda`。

```bash
python tools/artvip_kitchen.py download              # environment/ArtVIP/
```

未设置 `HF_ENDPOINT` 或 `--endpoint` 时，下载走 `https://hf-mirror.com`。厨房里的开关门、灯光和物理调整写在 artvip-usd 的 object USDA 里，见 `environment/artvip-usd/README.md`。机器人场景文件只保留摆放、物理变体和 GPU 配对缓冲。

---

## License

This package is licensed under the Apache License 2.0. See the [LICENSE](LICENSE) file for details.
