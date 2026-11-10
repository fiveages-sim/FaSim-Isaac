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
用法: ./sdg.sh <命令> [选项]

采集控制（不需要 Isaac python，直接跟运行中的仿真对话）:
  capture     交互式开始采集（选机器人/任务/相机/频率/输出目录）
  status      查看采集进度（帧数、物理步、错误数）
  stop        结束采集并写 session.json
  doctor      环境自检（路径/socket/扩展是否就绪）

配置与离线渲染（需要 Isaac python）:
  list        列出可用的机器人/任务
  describe    看某个任务的相机与 tracked_prims
  record      层2 录制
  render      层2 离线渲染（由 episode npz 出图）

示例:
  # 1) 启动仿真（另一个终端）
  cd ~/FaSim-Isaac && ./run.sh

  # 2) 开始采集
  ./scripts/sdg.sh capture

  # 3) 跑编排（另一个终端），然后结束
  ./scripts/sdg.sh stop
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

need_isaac_py() {
    [ -x "$ISAAC_PY" ] || die "找不到 Isaac python: $ISAAC_PY（设 ISAACSIM_DIR 或装 Isaac Sim）"
}

CMD="${1:-}"
[ $# -gt 0 ] && shift

case "$CMD" in
    capture|status|stop|doctor)
        PY="$(pick_python)" || die "找不到带 PyYAML 的 python3（设 SDG_PYTHON）"
        export PYTHONPATH="${EXT_ROOT}${PYTHONPATH:+:${PYTHONPATH}}"
        exec "$PY" "${SCRIPT_DIR}/cli.py" "$CMD" "$@"
        ;;
    list|describe|record|render)
        need_isaac_py
        export PYTHONPATH="${EXT_ROOT}${PYTHONPATH:+:${PYTHONPATH}}"
        exec "$ISAAC_PY" -m fasim.replicator.sdg "$CMD" "$@"
        ;;
    ""|-h|--help|help)
        usage; exit 0
        ;;
    *)
        echo -e "${RED}[ERROR]${NC} 未知命令: $CMD" >&2
        echo
        usage
        exit 1
        ;;
esac
