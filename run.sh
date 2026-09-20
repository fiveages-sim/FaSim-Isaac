#!/bin/bash

# FaSim-Isaac 启动脚本
# 配置见 config/fa_sim.conf（本机覆盖：config/fa_sim.local.conf）

set -u

RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m'

print_info() { echo -e "${GREEN}[INFO]${NC} $1"; }
print_warn() { echo -e "${YELLOW}[WARN]${NC} $1"; }
print_error() { echo -e "${RED}[ERROR]${NC} $1"; }

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
FA_SIM_CONFIG="${SCRIPT_DIR}/config/fa_sim.conf"
FA_SIM_LOCAL_CONFIG="${SCRIPT_DIR}/config/fa_sim.local.conf"

if [ ! -f "$FA_SIM_CONFIG" ]; then
    print_error "找不到配置文件: $FA_SIM_CONFIG"
    exit 1
fi
# shellcheck source=config/fa_sim.conf
source "$FA_SIM_CONFIG"
if [ -f "$FA_SIM_LOCAL_CONFIG" ]; then
    # shellcheck source=/dev/null
    source "$FA_SIM_LOCAL_CONFIG"
    print_info "已加载本机覆盖: config/fa_sim.local.conf"
fi

# 设置 CPU 性能模式
# RUN_SET_CPU_PERFORMANCE: 1=强制 / 0=跳过 / auto=尽力设置（容器内不 apt 安装）
is_container_env() {
    [ -f /.dockerenv ] && return 0
    [ -f /run/.containerenv ] && return 0
    [ -n "${CONTAINER_ID:-}" ] && return 0
    [ -n "${DISTROBOX_ENTER_PATH:-}" ] && return 0
    if command -v systemd-detect-virt >/dev/null 2>&1; then
        systemd-detect-virt -q -c 2>/dev/null && return 0
    fi
    return 1
}

set_cpu_performance_sysfs() {
    local gov_file set_count=0
    shopt -s nullglob
    for gov_file in /sys/devices/system/cpu/cpu*/cpufreq/scaling_governor; do
        if [ -w "$gov_file" ]; then
            if echo performance >"$gov_file" 2>/dev/null; then
                set_count=$((set_count + 1))
            fi
        elif command -v sudo >/dev/null 2>&1; then
            if echo performance | sudo tee "$gov_file" >/dev/null 2>&1; then
                set_count=$((set_count + 1))
            fi
        fi
    done
    shopt -u nullglob
    [ "$set_count" -gt 0 ]
}

set_cpu_performance() {
    local mode="${RUN_SET_CPU_PERFORMANCE:-auto}"
    local in_container=0
    is_container_env && in_container=1

    case "$mode" in
        0|false|no|off)
            print_info "已跳过 CPU 性能模式设置（RUN_SET_CPU_PERFORMANCE=$mode）"
            return 0
            ;;
    esac

    print_info "尝试设置 CPU 调频策略为 performance..."
    if set_cpu_performance_sysfs; then
        print_info "✓ 已通过 sysfs 设置 CPU performance"
        return 0
    fi

    if command -v cpupower >/dev/null 2>&1; then
        if sudo cpupower frequency-set -g performance; then
            print_info "✓ 已通过 cpupower 设置 CPU performance"
            return 0
        fi
        print_warn "cpupower 设置失败，跳过"
        return 0
    fi

    # 强制模式且非容器：尝试安装工具；容器内内核多为宿主机，apt 装 linux-tools 通常无效
    if [ "$mode" = "1" ] && [ "$in_container" -eq 0 ]; then
        print_warn "未找到 cpupower，尝试安装 linux-tools..."
        if sudo apt install -y linux-tools-common linux-tools-"$(uname -r)" 2>/dev/null \
            && command -v cpupower >/dev/null 2>&1 \
            && sudo cpupower frequency-set -g performance; then
            print_info "✓ 已安装并设置 CPU performance"
            return 0
        fi
        print_warn "安装或设置失败，跳过 CPU 性能模式设置"
        return 0
    fi

    if [ "$in_container" -eq 1 ]; then
        print_warn "当前在容器/distrobox 中，CPU 调频由宿主机控制，已跳过"
        print_warn "如需开启 performance，请在宿主机执行："
        echo -e "  ${GREEN}sudo cpupower frequency-set -g performance${NC}"
        print_warn "或在 config/fa_sim.local.conf 设置 RUN_SET_CPU_PERFORMANCE=0 以静默跳过"
    else
        print_warn "无法设置 CPU performance（无 sysfs 写权限且无 cpupower），已跳过"
    fi
}

set_cpu_performance

cleanup_zenoh() {
    if [ -n "${ZENOH_PID:-}" ]; then
        print_info "正在停止 Zenoh router（进程组 $ZENOH_PID）..."
        kill -TERM -"$ZENOH_PID" 2>/dev/null
        sleep 0.5
        kill -KILL -"$ZENOH_PID" 2>/dev/null
        wait "$ZENOH_PID" 2>/dev/null
        print_info "Zenoh router 已停止"
    fi
}

if [ "${RMW_IMPLEMENTATION:-}" = "rmw_zenoh_cpp" ]; then
    print_info "检测到 RMW_IMPLEMENTATION=rmw_zenoh_cpp，启动 Zenoh router..."
    setsid ros2 run rmw_zenoh_cpp rmw_zenohd &
    ZENOH_PID=$!
    print_info "Zenoh router 已在后台启动（进程组 PID: $ZENOH_PID）"
    trap cleanup_zenoh EXIT INT TERM
    sleep 1
else
    print_warn "当前未使用 rmw_zenoh_cpp（RMW_IMPLEMENTATION=${RMW_IMPLEMENTATION:-未设置}）"
    print_warn "如需使用 Zenoh，请先设置环境变量："
    echo -e "  ${GREEN}export RMW_IMPLEMENTATION=rmw_zenoh_cpp${NC}"
fi

if [ ! -d "${ISAACSIM_DIR}" ]; then
    print_error "未找到目录: ${ISAACSIM_DIR}，请确认 Isaac Sim 安装路径。"
    print_info "可在 config/fa_sim.local.conf 中设置 ISAACSIM_DIR，或："
    print_info "  ISAACSIM_DIR=/path/to/isaacsim ./run.sh"
    exit 1
fi

# 用法: ./run.sh [模式] [实例序号]
#   模式: 1=PhysX  2=Newton  3=Headless Streaming  q=退出
#   实例序号仅对 Headless 生效；也可用 FA_SIM_INSTANCE=N
CLI_MODE="${1:-}"
CLI_INSTANCE="${2:-}"

is_nonneg_int() {
    [[ "${1:-}" =~ ^[0-9]+$ ]]
}

resolve_streaming_instance() {
    local instance="${FA_SIM_INSTANCE:-0}"
    if [ -n "${CLI_INSTANCE}" ]; then
        instance="${CLI_INSTANCE}"
    elif [ -z "${CLI_MODE}" ]; then
        read -r -p "Headless 实例序号 FA_SIM_INSTANCE [${instance}]: " instance_input
        if [ -n "${instance_input}" ]; then
            instance="${instance_input}"
        fi
    fi
    if ! is_nonneg_int "${instance}"; then
        print_error "实例序号必须是非负整数，当前: ${instance}"
        exit 1
    fi
    FA_SIM_INSTANCE="${instance}"
}

apply_streaming_isolation() {
    local signal_port stream_port domain_id
    signal_port=$((STREAM_SIGNAL_PORT_BASE + FA_SIM_INSTANCE * STREAM_PORT_STEP))
    stream_port=$((STREAM_STREAM_PORT_BASE + FA_SIM_INSTANCE * STREAM_PORT_STEP))
    domain_id=$((ROS_DOMAIN_ID_BASE + FA_SIM_INSTANCE))

    if [ "${domain_id}" -gt 232 ]; then
        print_error "ROS_DOMAIN_ID=${domain_id} 超出有效范围 0–232（BASE=${ROS_DOMAIN_ID_BASE}, INSTANCE=${FA_SIM_INSTANCE}）"
        exit 1
    fi

    export ROS_DOMAIN_ID="${domain_id}"
    STREAM_SIGNAL_PORT="${signal_port}"
    STREAM_STREAM_PORT="${stream_port}"

    print_info "Headless 多实例隔离已应用:"
    print_info "  FA_SIM_INSTANCE=${FA_SIM_INSTANCE}"
    print_info "  livestream signalPort(TCP)=${STREAM_SIGNAL_PORT}"
    print_info "  livestream streamPort(UDP)=${STREAM_STREAM_PORT}"
    print_info "  ROS_DOMAIN_ID=${ROS_DOMAIN_ID}"
}

if [ -z "${CLI_MODE}" ]; then
    echo ""
    echo "请选择 Isaac Sim 启动模式"
    echo
    echo "  [启动]"
    echo "    1) 正常启动 PhysX (${ISAACSIM_LAUNCH_NORMAL})"
    echo "    2) Newton 物理引擎 (${ISAACSIM_LAUNCH_NEWTON})"
    echo "    3) Headless Streaming (${ISAACSIM_LAUNCH_STREAMING}，仅仿真，不启动控制)"
    echo "       多实例: 端口/ROS_DOMAIN_ID 按 FA_SIM_INSTANCE 累加"
    echo
    echo "  [其他]"
    echo "    q) 退出"
    echo
    read -r -p "输入选项 [1-3/q]: " isaac_mode
else
    isaac_mode="${CLI_MODE}"
fi

cd "${ISAACSIM_DIR}" || { print_error "无法进入目录: ${ISAACSIM_DIR}"; exit 1; }

case "${isaac_mode:-1}" in
    1)
        if [ ! -x "./${ISAACSIM_LAUNCH_NORMAL}" ]; then
            print_error "未找到可执行文件 ${ISAACSIM_DIR}/${ISAACSIM_LAUNCH_NORMAL}"
            exit 1
        fi
        print_info "启动 Isaac Sim（正常 / PhysX）..."
        "./${ISAACSIM_LAUNCH_NORMAL}"
        ;;
    2)
        if [ ! -x "./${ISAACSIM_LAUNCH_NEWTON}" ]; then
            print_error "未找到可执行文件 ${ISAACSIM_DIR}/${ISAACSIM_LAUNCH_NEWTON}"
            print_info "Newton 启动脚本通常随 Isaac Sim 6.0+ 提供，请确认安装版本。"
            exit 1
        fi
        print_info "启动 Isaac Sim（Newton 物理引擎）..."
        "./${ISAACSIM_LAUNCH_NEWTON}"
        ;;
    3)
        if [ ! -x "./${ISAACSIM_LAUNCH_STREAMING}" ]; then
            print_error "未找到可执行文件 ${ISAACSIM_DIR}/${ISAACSIM_LAUNCH_STREAMING}，请确认脚本存在且有执行权限。"
            exit 1
        fi
        resolve_streaming_instance
        apply_streaming_isolation
        print_info "启动 Isaac Sim（Headless Streaming 模式）..."
        "./${ISAACSIM_LAUNCH_STREAMING}" \
            --/exts/omni.kit.livestream.app/primaryStream/signalPort="${STREAM_SIGNAL_PORT}" \
            --/exts/omni.kit.livestream.app/primaryStream/streamPort="${STREAM_STREAM_PORT}"
        ;;
    q|Q)
        echo "已退出。"
        exit 0
        ;;
    *)
        print_error "无效选项: ${isaac_mode}"
        print_info "用法: ./run.sh [1|2|3|q] [FA_SIM_INSTANCE]"
        exit 1
        ;;
esac
