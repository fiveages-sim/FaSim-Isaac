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
./init_repo.sh
```

脚本运行后首先选择操作类型：

#### 操作 1：初始化仓库（拉取子模块并切换到目标分支）

提供三种初始化模式：

- **模式 1（默认）**：仅拉取 public 资产，适用于外部用户，无需内部仓库访问权限
- **模式 2**：拉取全部资产（含 private），需要内部仓库访问权限
- **模式 3（W2）**：顶层子模块全部拉取，`robots` 目录使用稀疏检出只保留 W2 相关目录（FiveAges_W2、Marvin、Dual_Stand1、sensors、grippers、dexhands），并仅初始化对应的 private 嵌套子模块

脚本会自动完成子模块初始化、分支切换并更新到最新提交。

#### 操作 2：配置环境

- **Isaac ROS2 Jazzy Workspace**：自动下载 IsaacSim 5.1.0 的 ROS workspaces，提取 `jazzy_ws` 到 `isaac_jazzy_ws/`，安装 rosdep / colcon 依赖，构建工作空间并写入 `~/.bashrc`

### 3. 启动 Isaac Sim

```bash
./run.sh
```

`run.sh` 会依次执行：

1. 将 CPU 调频策略设置为 performance（需要 sudo 权限）
2. 若环境变量 `RMW_IMPLEMENTATION=rmw_zenoh_cpp`，自动启动 Zenoh router 并在退出时清理
3. 启动 `~/isaacsim/isaac-sim.sh`

---

## 文件结构

```
FaSim-Isaac/
├── init_repo.sh                  # 一键初始化脚本
├── run.sh                        # Isaac Sim 启动脚本
├── submodules_visibility.conf    # 嵌套子模块 public / private 配置
├── robots/                       # 机器人 USD 资产（Git 子模块）
├── environment/
│   ├── fiveages_env/             # 场景 USD 资产（Git 子模块）
│   └── fa-project-usd/           # Fa Project USD 资产（Git 子模块，private）
└── isaac_jazzy_ws/               # ROS2 Jazzy 工作空间（由 init_repo.sh 操作 2 生成）
```

`robots` 内按类型分子目录：人形（humannoid）、机械臂（manipulators）、夹爪（grippers）、灵巧手（dexhands）、传感器（sensors）、支架（stands）等；`environment` 下为场景与项目相关 USD。具体机型与文件名可直接在对应目录中查看。

---

