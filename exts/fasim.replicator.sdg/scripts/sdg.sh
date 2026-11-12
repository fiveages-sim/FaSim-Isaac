#!/bin/bash
# FaSim 合成数据采集 —— 统一入口。
#
#   ./sdg.sh capture     交互式开始采集（推荐）
#   ./sdg.sh status      查看进度
#   ./sdg.sh stop        结束采集
#   ./sdg.sh doctor      环境自检
#
#   ./sdg.sh list        列出机器人/任务配置
#   ./sdg.sh describe    看某个任务的相机/tracked_prims
#   ./sdg.sh render      层2 离线渲染（需要 Isaac python）
#   ./sdg.sh record      层2 录制（需要 Isaac python）
#
# 前 4 条（capture/status/stop/doctor）**不需要 Isaac python** —— 它们只是
# 通过 Unix socket 跟运行中的 Isaac 进程对话，用系统 python3 即可。
# 用户不必 source 任何环境。
#
# 后几条（render/record/describe）走 ``isaacsim/python.sh``。

set -uo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
EXT_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"
ISAACSIM_DIR="${ISAACSIM_DIR:-${HOME}/isaacsim}"
ISAAC_PY="${ISAACSIM_DIR}/python.sh"

RED='\033[0;31m'; GREEN='\033[0;32m'; YELLOW='\033[1;33m'; NC='\033[0m'
die() { echo -e "${RED}[ERROR]${NC} $*" >&2; exit 1; }
warn() { echo -e "${YELLOW}[WARN]${NC} $*" >&2; }

usage() {
    cat <<'EOF'
用法: ./sdg.sh [命令] [选项]

  ./sdg.sh              ← 交互式主菜单（推荐日常使用，会记住上次选择）
  ./sdg.sh <命令> ...   ← 参数化调用（适合脚本/CI）

主菜单功能:
  采集 / 查看进度 / 结束采集 / 数据工具 / 回放 / 自检 / 切换机器人任务

子命令（参数化）:
采集控制（不需要 Isaac python，直接跟运行中的仿真对话）:
  capture     开始采集（不指定参数则交互）
              --robot <key> --task <key> --what camera,joint,ee,object
              --camera all|<name,...> --every 10 --out <dir>
  status      查看采集进度（帧数、物理步、错误数）
  stop        结束采集并写 session.json
  doctor      环境自检（路径/socket/扩展是否就绪）

数据检查与转换（不需要 Isaac python）:
  info        看采集会话里有什么（帧数、相机、关节数、体积）
  pack        采集会话 → episode npz（层2 能读的格式）
              pack <session> [--robot <key>]   反算末端指令位姿需要 robot
  trim        切成任务片段，剔除待机段（ACT 训练必需）
  fk          用 FK 算末端真值（需要 pinocchio）
              fk <episode.npz>                     体检采集
              fk <*.replay_fN.npz> --source <ep>   体检回放
  diag        一键体检 episode（末端/关节/时间轴/物体/相机）
  fk-diag     体检回放（回放 vs 编排，mm 级 + 方向）
  shots       相机联络图：把会话的相机帧拼成一张图，人工检查 RGB
              shots <session> [--segment] [--camera head] [--cols 8 --rows 6]

回放自检（需要 Isaac python）:
  live        在同一 Isaac 实例里回放 episode
              --drive joints|ee  --joint-src cmd|actual  --wait
  preview     层2 离线预览（弹窗口）
  live-status / live-stop / live-pause / live-resume

配置与离线渲染（需要 Isaac python）:
  list        列出可用的机器人/任务
  describe    看某个任务的相机与 tracked_prims
  record      层2 录制
  render      层2 离线渲染（由 episode npz 出图）

配置隔离:
  所有机器人与任务信息都在 configs/<robot>/ 下，本脚本不含任何具体机器人名。
    新增机器人 → 新建 configs/<robot>/robot.yaml + tasks/<task>.yaml，不改代码

示例:
  # 交互式（最常用）
  ./scripts/sdg.sh

  # 参数化采集（脚本/CI）
  ./scripts/sdg.sh capture --robot <robot_key> --task <task_key> \
      --what camera,joint,ee,object --camera all --every 10 --out ~/fasim_data/run_01

  # 转格式 + 体检 + 拼图
  ./scripts/sdg.sh pack ~/fasim_data/run_01 --robot <robot_key>
  ./scripts/sdg.sh diag ~/fasim_data/run_01/episode.npz
  ./scripts/sdg.sh shots ~/fasim_data/run_01 --segment
EOF
}

# 系统 python：CLI 只用到标准库 + PyYAML（读任务配置）
pick_python() {
    local cand
    for cand in "${SDG_PYTHON:-}" "$(command -v python3)" /usr/bin/python3; do
        [ -n "$cand" ] || continue
        [ -x "$cand" ] || continue
        if "$cand" -c "import yaml" >/dev/null 2>&1; then
            echo "$cand"; return 0
        fi
    done
    for cand in /home/fiveages/miniconda3/envs/lerobot-ros2/bin/python; do
        [ -x "$cand" ] && "$cand" -c "import yaml" >/dev/null 2>&1 && { echo "$cand"; return 0; }
    done
    return 1
}

# 带 pinocchio 的 python：`fk` 做正解需要它（Isaac 的 python 里没有，
# 系统 python3 通常也没有）。按优先级找一个能 import pinocchio 的。
pick_fk_python() {
    local cand
    for cand in "${SDG_FK_PYTHON:-}" \
                /home/fiveages/miniconda3/envs/lerobot-ros2/bin/python \
                "$(command -v python3)" /usr/bin/python3; do
        [ -n "$cand" ] || continue
        [ -x "$cand" ] || continue
        if "$cand" -c "import pinocchio, numpy, yaml" >/dev/null 2>&1; then
            echo "$cand"; return 0
        fi
    done
    return 1
}

need_isaac_py() {
    [ -x "$ISAAC_PY" ] || die "找不到 Isaac python: $ISAAC_PY（设 ISAACSIM_DIR 或装 Isaac Sim）"
}

CMD="${1:-}"
[ $# -gt 0 ] && shift

case "$CMD" in
    fk|diag|fk-diag)
        # FK/体检需要 pinocchio —— Isaac 的 python 里没有，所以单独选解释器。
        PY="$(pick_fk_python)" || die "找不到带 pinocchio 的 python（设 SDG_FK_PYTHON）"
        export PYTHONPATH="${EXT_ROOT}${PYTHONPATH:+:${PYTHONPATH}}"
        exec "$PY" "${SCRIPT_DIR}/cli.py" "$CMD" "$@"
        ;;
    shots)
        # 相机联络图：只要 numpy（可选 imageio/PIL 出 PNG），不需要 Isaac。
        PY="$(pick_python)" || die "找不到带 numpy 的 python3（设 SDG_PYTHON）"
        exec "$PY" "${SCRIPT_DIR}/camera_shots.py" "$@"
        ;;
    menu)
        PY="$(pick_python)" || die "找不到带 PyYAML 的 python3（设 SDG_PYTHON）"
        export PYTHONPATH="${EXT_ROOT}${PYTHONPATH:+:${PYTHONPATH}}"
        exec "$PY" "${SCRIPT_DIR}/sdg_menu.py" "$@"
        ;;
    capture|status|stop|doctor|info|pack|trim|live|live-status|live-stop|live-pause|live-resume|list|export)
        PY="$(pick_python)" || die "找不到带 PyYAML 的 python3（设 SDG_PYTHON）"
        export PYTHONPATH="${EXT_ROOT}${PYTHONPATH:+:${PYTHONPATH}}"
        exec "$PY" "${SCRIPT_DIR}/cli.py" "$CMD" "$@"
        ;;
    preview|list|describe|record|render)
        need_isaac_py
        export PYTHONPATH="${EXT_ROOT}${PYTHONPATH:+:${PYTHONPATH}}"
        exec "$ISAAC_PY" "${SCRIPT_DIR}/cli.py" "$CMD" "$@"
        ;;
    "")
        # 无参数 = 交互式主菜单（推荐日常使用）
        exec "$0" menu
        ;;
    -h|--help|help)
        usage; exit 0
        ;;
    *)
        echo -e "${RED}[ERROR]${NC} 未知命令: $CMD" >&2
        echo
        usage
        exit 1
        ;;
esac
