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

print_info "启动 Isaac Sim..."
cd "$HOME/isaacsim" || { print_error "未找到目录: ~/isaacsim"; exit 1; }
./isaac-sim.sh
