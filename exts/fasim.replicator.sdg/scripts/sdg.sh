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
  capture     交互式开始采集（选机器人/任务/采集项/相机/频率/输出目录）
  status      查看采集进度（帧数、物理步、错误数）
  stop        结束采集并写 session.json
  doctor      环境自检（路径/socket/扩展是否就绪）

数据检查与转换（不需要 Isaac python）:
  info        看采集会话里有什么（帧数、相机、关节数、体积）
  pack        采集会话 → episode npz（层2 能读的格式）
  trim        切成任务片段，剔除待机段（ACT 训练必需）
  fk          用 FK 算末端真值（需要 pinocchio）
              fk <episode.npz>                     体检采集：末端是不是真采到了
              fk <*.replay_fN.npz> --source <ep>   体检回放：手偏了几毫米、偏哪

回放自检（需要 Isaac python，preview 的 --drive ee 还需要运控在跑）:
  preview     回放 episode 验证采集是否忠实
              --drive joints  逐帧驱关节（默认，不需 ROS）
              --drive ee      末端位姿发 /left_target,/right_target，OCS2 闭环

配置与离线渲染（需要 Isaac python）:
  list        列出可用的机器人/任务
  describe    看某个任务的相机与 tracked_prims
  record      层2 录制
  render      层2 离线渲染（由 episode npz 出图）

示例:
  # 1) 启动仿真（另一个终端）
  cd ~/FaSim-Isaac && ./run.sh

  # 2) 开始采集（可只采关节/末端，不采相机）
  ./scripts/sdg.sh capture
  ./scripts/sdg.sh capture --what joint,ee --out ~/fasim_data/calib

  # 3) 跑编排（另一个终端），然后结束
  ./scripts/sdg.sh stop

  # 4) 转成 episode 并回放自检
  ./scripts/sdg.sh pack ~/fasim_data/.../20261110-1121
  ./scripts/sdg.sh preview ~/fasim_data/.../20261110-1121/episode.npz --gui
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
    fk)
        # FK 需要 pinocchio —— Isaac 的 python 里没有，所以单独选解释器。
        PY="$(pick_fk_python)" || die "找不到带 pinocchio 的 python（设 SDG_FK_PYTHON）"
        export PYTHONPATH="${EXT_ROOT}${PYTHONPATH:+:${PYTHONPATH}}"
        exec "$PY" "${SCRIPT_DIR}/cli.py" fk "$@"
        ;;
    capture|status|stop|doctor|info|pack|trim|live|live-status|live-stop|live-pause|live-resume)
        PY="$(pick_python)" || die "找不到带 PyYAML 的 python3（设 SDG_PYTHON）"
        export PYTHONPATH="${EXT_ROOT}${PYTHONPATH:+:${PYTHONPATH}}"
        exec "$PY" "${SCRIPT_DIR}/cli.py" "$CMD" "$@"
        ;;
    preview|list|describe|record|render)
        need_isaac_py
        export PYTHONPATH="${EXT_ROOT}${PYTHONPATH:+:${PYTHONPATH}}"
        exec "$ISAAC_PY" "${SCRIPT_DIR}/cli.py" "$CMD" "$@"
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
