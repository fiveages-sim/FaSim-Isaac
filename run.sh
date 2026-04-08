#!/bin/bash

# FaSim-Isaac 启动脚本

set -u

RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m'

print_info() { echo -e "${GREEN}[INFO]${NC} $1"; }
print_warn() { echo -e "${YELLOW}[WARN]${NC} $1"; }
print_error() { echo -e "${RED}[ERROR]${NC} $1"; }

# 设置 CPU 性能模式
if command -v cpupower >/dev/null 2>&1; then
    print_info "设置 CPU 调频策略为 performance..."
    sudo cpupower frequency-set -g performance || print_warn "cpupower 设置失败，跳过"
else
    print_warn "未找到 cpupower，尝试安装 linux-tools-common..."
    if sudo apt install -y linux-tools-common linux-tools-"$(uname -r)" 2>/dev/null; then
        print_info "设置 CPU 调频策略为 performance..."
        sudo cpupower frequency-set -g performance || print_warn "cpupower 设置失败，跳过"
    else
        print_warn "安装失败，跳过 CPU 性能模式设置"
    fi
fi

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

ISAACSIM_DIR="$HOME/isaacsim"

if [ ! -d "${ISAACSIM_DIR}" ]; then
    print_error "未找到目录: ~/isaacsim，请确认 Isaac Sim 安装路径。"
    exit 1
fi

echo ""
echo "请选择 Isaac Sim 启动模式:"
echo "  1) 正常启动 (isaac-sim.sh)"
echo "  2) Headless Streaming 模式 (isaac-sim.streaming.sh，仅仿真，不启动控制)"
echo ""
read -r -p "请输入选项 [1-2] (默认: 1): " isaac_mode
if [ -z "${isaac_mode}" ]; then
    isaac_mode="1"
fi

cd "${ISAACSIM_DIR}" || { print_error "无法进入目录: ~/isaacsim"; exit 1; }

case "${isaac_mode}" in
    1)
        print_info "启动 Isaac Sim（正常模式）..."
        ./isaac-sim.sh
        ;;
    2)
        if [ ! -x "./isaac-sim.streaming.sh" ]; then
            print_error "未找到可执行文件 ~/isaacsim/isaac-sim.streaming.sh，请确认脚本存在且有执行权限。"
            exit 1
        fi
        print_info "启动 Isaac Sim（Headless Streaming 模式）..."
        ./isaac-sim.streaming.sh
        ;;
    *)
        print_error "无效选项: ${isaac_mode}"
        exit 1
        ;;
esac
