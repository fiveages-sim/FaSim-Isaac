# FaSim-Isaac

FaSim-Isaac 是第五纪仿真平台的 Isaac Sim 资产快速部署仓库，旨在通过一键脚本将所有仿真所需的机器人 USD 模型与场景环境资产拉取到本地，方便用户快速搭建 Isaac Sim 仿真环境。

## 快速开始

### 1. 克隆主仓库

```bash
git clone git@github.com:fiveages-sim/FaSim-Isaac.git
cd FaSim-Isaac
```

### 2. 运行初始化脚本

```bash
bash init_repo.sh
```

运行后会提示选择初始化模式：

- **选项 1（默认）**：仅拉取 public 资产，适用于外部用户，无需内部仓库访问权限
- **选项 2**：拉取全部资产（含 private），需要内部仓库访问权限

脚本会自动完成子模块初始化、分支切换并更新到最新提交。

---

## 文件结构

```
FaSim-Isaac/
├── init_repo.sh                  # 一键初始化脚本
├── submodules_visibility.conf    # 嵌套子模块 public / private 配置
├── robot_usds/                   # 机器人 USD 资产（Git 子模块）
└── environment/
    ├── fiveages_env/             # 场景 USD 资产（Git 子模块）
    └── fa-project-usd/           # Fa Project USD 资产（Git 子模块）
```

`robot_usds` 内按类型分子目录：人形、机械臂、夹爪、灵巧手、移动底盘、移动机械臂、传感器等；`environment` 下为场景与项目相关 USD。具体机型与文件名可直接在对应目录中查看。

---

