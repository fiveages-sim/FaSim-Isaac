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

#### 操作 2：配置环境

- **Isaac ROS2 Jazzy Workspace**：可选 Isaac Sim 版本（默认与版本列表见 `config/fa_sim.conf`），下载对应 ROS workspaces，提取 `jazzy_ws` 到 `isaac_jazzy_ws/`，安装 rosdep / colcon 依赖，构建工作空间并写入 `~/.bashrc`。也可通过环境变量指定：`ISAAC_SIM_VERSION=6.0.1 ./init.sh`

### 3. 启动 Isaac Sim

```bash
./run.sh
```

`run.sh` 会依次执行：

1. 尝试将 CPU 调频设为 performance（默认 `auto`：容器/distrobox 内不 apt 安装；可在 `config/fa_sim.local.conf` 设为 `0` 跳过）
2. 若环境变量 `RMW_IMPLEMENTATION=rmw_zenoh_cpp`，自动启动 Zenoh router 并在退出时清理
3. 选择启动模式：PhysX 正常启动 / Newton 物理引擎 / Headless Streaming

### 配置说明

与 `fa_w2_ws` 类似，脚本共用 bash 配置文件：

| 文件 | 说明 |
|------|------|
| `config/fa_sim.conf` | 仓库默认配置（提交 git）：Isaac 路径、版本列表、apt 依赖等 |
| `config/fa_sim.local.conf` | 本机覆盖（gitignore）：复制 `fa_sim.local.template.conf` 后修改 |

```bash
cp config/fa_sim.local.template.conf config/fa_sim.local.conf
# 编辑 ISAACSIM_DIR / ISAAC_SIM_DEFAULT_VERSION 等
```

---

## 文件结构

```
FaSim-Isaac/
├── init.sh                       # 一键初始化脚本
├── run.sh                        # Isaac Sim 启动脚本
├── config/
│   ├── fa_sim.conf               # 共用配置（init / run）
│   └── fa_sim.local.template.conf
├── submodules_visibility.conf    # 嵌套子模块 public / private 配置
├── robots/                       # 机器人 USD 资产（Git 子模块）
├── environment/
│   ├── fiveages_env/             # 场景 USD 资产（Git 子模块）
│   └── fa-project-usd/           # Fa Project USD 资产（Git 子模块，private）
└── isaac_jazzy_ws/               # ROS2 Jazzy 工作空间（由 init.sh 操作 2 生成）
```

`robots` 内按类型分子目录：人形（humanoid）、机械臂（manipulators）、夹爪（grippers）、灵巧手（dexhands）、传感器（sensors）、支架（stands）等；`environment` 下为场景与项目相关 USD。具体机型与文件名可直接在对应目录中查看。

---
