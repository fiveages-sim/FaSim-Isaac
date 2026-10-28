#!/bin/bash

# FaSim-Isaac 启动脚本（普通 GUI）
# Headless：./run_headless.sh
# 配置：config/fa_sim.conf + config/fa_sim.local.conf（交互选项会写回 local）

set -u

RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
CYAN='\033[0;36m'
NC='\033[0m'

print_info() { echo -e "${GREEN}[INFO]${NC} $1"; }
print_warn() { echo -e "${YELLOW}[WARN]${NC} $1"; }
print_error() { echo -e "${RED}[ERROR]${NC} $1"; }

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
FA_SIM_CONFIG="${SCRIPT_DIR}/config/fa_sim.conf"
FA_SIM_LOCAL_CONFIG="${SCRIPT_DIR}/config/fa_sim.local.conf"

HEADLESS=0
CLI_PHYSICS=""
CLI_CPU=""
CLI_ROS_WS=""
CLI_INSTANCE=""
NONINTERACTIVE=0

usage() {
    cat <<'EOF'
用法:
  ./run.sh              # 普通 GUI 启动（交互选配置）
  ./run_headless.sh     # Headless Streaming（交互选配置）

选项（可跳过交互）:
  --headless            Headless Streaming
  --physics physx|newton
  --cpu 0|1|auto        CPU performance（0=跳过）
  --ros-ws PATH         默认加载的 ROS2 工作空间（含 install/setup.bash）
  --instance N          Headless 实例序号（端口 / ROS_DOMAIN_ID）
  -y, --yes             非交互：全部用配置/默认值
  -h, --help
EOF
}

while [ $# -gt 0 ]; do
    case "$1" in
        --headless) HEADLESS=1; shift ;;
        --physics)
            CLI_PHYSICS="${2:-}"; shift 2
            ;;
        --cpu)
            CLI_CPU="${2:-}"; shift 2
            ;;
        --ros-ws)
            CLI_ROS_WS="${2:-}"; shift 2
            ;;
        --instance)
            CLI_INSTANCE="${2:-}"; shift 2
            ;;
        -y|--yes) NONINTERACTIVE=1; shift ;;
        -h|--help) usage; exit 0 ;;
        -*)
            print_error "未知选项: $1"
            usage
            exit 1
            ;;
        *)
            print_error "多余参数: $1（旧版数字模式已弃用，请用 ./run_headless.sh --instance N）"
            usage
            exit 1
            ;;
    esac
done

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

# 交互默认值（local.conf / 环境变量可覆盖）
RUN_PHYSICS_BACKEND="${RUN_PHYSICS_BACKEND:-physx}"
RUN_SET_CPU_PERFORMANCE="${RUN_SET_CPU_PERFORMANCE:-auto}"
RUN_ROS2_WS="${RUN_ROS2_WS:-${HOME}/ros2_ws}"
RUN_ISAAC_ROS_WS="${RUN_ISAAC_ROS_WS:-${SCRIPT_DIR}/${ISAAC_JAZZY_WS_NAME:-isaac_jazzy_ws}}"
RUN_SAVE_LOCAL="${RUN_SAVE_LOCAL:-1}"

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
    else
        print_warn "无法设置 CPU performance（无 sysfs 写权限且无 cpupower），已跳过"
    fi
}

is_nonneg_int() {
    [[ "${1:-}" =~ ^[0-9]+$ ]]
}

prompt_choice() {
    # $1=prompt $2=default → writes answer to REPLY
    local prompt="$1"
    local default="$2"
    if [ "$NONINTERACTIVE" -eq 1 ]; then
        REPLY="$default"
        return 0
    fi
    read -r -p "${prompt} [${default}]: " REPLY
    if [ -z "${REPLY}" ]; then
        REPLY="$default"
    fi
}

normalize_physics() {
    case "${1,,}" in
        1|physx|normal|p) echo physx ;;
        2|newton|n) echo newton ;;
        *) echo "" ;;
    esac
}

normalize_cpu() {
    case "${1,,}" in
        0|false|no|off|n) echo 0 ;;
        1|true|yes|on|y) echo 1 ;;
        auto|a) echo auto ;;
        *) echo "" ;;
    esac
}

resolve_ros_ws_setup() {
    local root="$1"
    if [ -z "$root" ] || [ "$root" = "none" ] || [ "$root" = "-" ]; then
        echo ""
        return 0
    fi
    if [ -f "${root}/install/setup.bash" ]; then
        echo "${root}/install/setup.bash"
        return 0
    fi
    if [ -f "${root}/setup.bash" ]; then
        echo "${root}/setup.bash"
        return 0
    fi
    echo ""
}

save_local_conf() {
    [ "${RUN_SAVE_LOCAL}" = "1" ] || return 0
    local tmp
    tmp="$(mktemp)"
    {
        echo "# 本机覆盖（由 ./run.sh 交互写入；可手改）"
        echo "# 生成时间: $(date -Iseconds 2>/dev/null || date)"
        echo
        if [ -n "${ISAACSIM_DIR:-}" ]; then
            echo "ISAACSIM_DIR=\"${ISAACSIM_DIR}\""
        fi
        echo "RUN_PHYSICS_BACKEND=\"${RUN_PHYSICS_BACKEND}\""
        echo "RUN_SET_CPU_PERFORMANCE=\"${RUN_SET_CPU_PERFORMANCE}\""
        echo "RUN_ROS2_WS=\"${RUN_ROS2_WS}\""
        echo "RUN_ISAAC_ROS_WS=\"${RUN_ISAAC_ROS_WS}\""
        if [ -f "$FA_SIM_LOCAL_CONFIG" ]; then
            # 保留用户其它键（非本次管理的）；丢掉会破坏 ROS/Kit 的 PYTHONPATH 清场行
            while IFS= read -r line || [ -n "$line" ]; do
                case "$line" in
                    ''|\#*) continue ;;
                    ISAACSIM_DIR=*|RUN_PHYSICS_BACKEND=*|RUN_SET_CPU_PERFORMANCE=*|RUN_ROS2_WS=*|RUN_ISAAC_ROS_WS=*|RUN_SAVE_LOCAL=*)
                        continue
                        ;;
                    unset\ PYTHONPATH*|unset\ PYTHONPATH|PYTHONPATH=*)
                        continue
                        ;;
                esac
                echo "$line"
            done <"$FA_SIM_LOCAL_CONFIG"
        fi

    } >"$tmp"
    mv "$tmp" "$FA_SIM_LOCAL_CONFIG"
    print_info "已保存本机默认到 config/fa_sim.local.conf"
}

print_config_summary() {
    local title="${1:-已保存的默认配置}"
    echo -e "${CYAN}${title}:${NC}"
    echo "  物理:     ${RUN_PHYSICS_BACKEND}"
    echo "  CPU:      ${RUN_SET_CPU_PERFORMANCE}"
    echo "  ROS2 WS:  ${RUN_ROS2_WS}"
    if [ "$HEADLESS" -eq 1 ]; then
        echo "  Instance: ${FA_SIM_INSTANCE:-0}"
    fi
}

apply_cli_overrides() {
    local ans
    if [ -n "$CLI_PHYSICS" ]; then
        ans="$(normalize_physics "$CLI_PHYSICS")"
        if [ -z "$ans" ]; then
            print_error "无效 --physics: $CLI_PHYSICS（用 physx|newton）"
            exit 1
        fi
        RUN_PHYSICS_BACKEND="$ans"
    fi
    if [ -n "$CLI_CPU" ]; then
        ans="$(normalize_cpu "$CLI_CPU")"
        if [ -z "$ans" ]; then
            print_error "无效 --cpu: $CLI_CPU（用 0|1|auto）"
            exit 1
        fi
        RUN_SET_CPU_PERFORMANCE="$ans"
    fi
    if [ -n "$CLI_ROS_WS" ]; then
        RUN_ROS2_WS="$CLI_ROS_WS"
    fi
    if [ -n "${CLI_INSTANCE}" ]; then
        if ! is_nonneg_int "${CLI_INSTANCE}"; then
            print_error "实例序号必须是非负整数，当前: ${CLI_INSTANCE}"
            exit 1
        fi
        FA_SIM_INSTANCE="${CLI_INSTANCE}"
    fi
}

prompt_reselect_config() {
    local phys_default cpu_default ros_default ans instance

    # --- Physics ---
    phys_default="${RUN_PHYSICS_BACKEND}"
    echo "物理引擎:"
    echo "  1) PhysX（普通）"
    echo "  2) Newton"
    prompt_choice "选择 [1/2]" "$( [ "$phys_default" = newton ] && echo 2 || echo 1 )"
    ans="$(normalize_physics "$REPLY")"
    if [ -z "$ans" ]; then
        print_error "无效选项: $REPLY"
        exit 1
    fi
    RUN_PHYSICS_BACKEND="$ans"

    # --- CPU ---
    cpu_default="${RUN_SET_CPU_PERFORMANCE}"
    echo ""
    echo "CPU performance 调频:"
    echo "  1) 开启（强制 1）"
    echo "  2) 自动 auto（推荐；容器内跳过 apt）"
    echo "  3) 关闭（0）"
    case "$cpu_default" in
        1) prompt_choice "选择 [1/2/3]" "1" ;;
        0) prompt_choice "选择 [1/2/3]" "3" ;;
        *) prompt_choice "选择 [1/2/3]" "2" ;;
    esac
    case "$REPLY" in
        1) RUN_SET_CPU_PERFORMANCE=1 ;;
        3) RUN_SET_CPU_PERFORMANCE=0 ;;
        2|*) RUN_SET_CPU_PERFORMANCE=auto ;;
    esac

    # --- ROS2 workspace ---
    ros_default="${RUN_ROS2_WS}"
    echo ""
    echo "默认加载的 ROS2 工作空间（robot_common_launch / *_description 所在）:"
    echo "  1) ${HOME}/ros2_ws"
    echo "  2) ${SCRIPT_DIR}/isaac_jazzy_ws（仅 Isaac 官方 jazzy 包）"
    echo "  3) 自定义路径"
    echo "  4) 不 source（none）"
    echo "  当前默认: ${ros_default}"
    case "$ros_default" in
        "${HOME}/ros2_ws"|"$HOME/ros2_ws") prompt_choice "选择 [1-4]" "1" ;;
        *isaac_jazzy_ws*) prompt_choice "选择 [1-4]" "2" ;;
        none|-|"") prompt_choice "选择 [1-4]" "4" ;;
        *) prompt_choice "选择 [1-4]" "3" ;;
    esac
    case "$REPLY" in
        1) RUN_ROS2_WS="${HOME}/ros2_ws" ;;
        2) RUN_ROS2_WS="${SCRIPT_DIR}/isaac_jazzy_ws" ;;
        3)
            prompt_choice "输入工作空间根目录" "$ros_default"
            RUN_ROS2_WS="$REPLY"
            ;;
        4) RUN_ROS2_WS="none" ;;
        *)
            print_error "无效选项: $REPLY"
            exit 1
            ;;
    esac

    if [ "$HEADLESS" -eq 1 ]; then
        instance="${FA_SIM_INSTANCE:-0}"
        echo ""
        prompt_choice "Headless 实例序号 FA_SIM_INSTANCE" "$instance"
        instance="$REPLY"
        if ! is_nonneg_int "${instance}"; then
            print_error "实例序号必须是非负整数，当前: ${instance}"
            exit 1
        fi
        FA_SIM_INSTANCE="${instance}"
    fi
}

interactive_config() {
    local ans reselect=0

    # 规范化已加载默认
    ans="$(normalize_physics "${RUN_PHYSICS_BACKEND}")"
    RUN_PHYSICS_BACKEND="${ans:-physx}"
    ans="$(normalize_cpu "${RUN_SET_CPU_PERFORMANCE}")"
    RUN_SET_CPU_PERFORMANCE="${ans:-auto}"
    FA_SIM_INSTANCE="${FA_SIM_INSTANCE:-0}"

    apply_cli_overrides

    echo ""
    echo -e "${CYAN}══ FaSim-Isaac 启动配置 ══${NC}"
    if [ "$HEADLESS" -eq 1 ]; then
        echo -e "  模式: ${GREEN}Headless Streaming${NC}"
    else
        echo -e "  模式: ${GREEN}普通 GUI${NC}"
    fi
    echo ""
    print_config_summary "已保存的默认配置"
    if [ -f "$FA_SIM_LOCAL_CONFIG" ]; then
        echo -e "  ${YELLOW}(来自 config/fa_sim.local.conf)${NC}"
    else
        echo -e "  ${YELLOW}(来自 config/fa_sim.conf；尚未保存本机默认)${NC}"
    fi
    echo ""

    if [ "$NONINTERACTIVE" -eq 1 ]; then
        reselect=0
    elif [ -n "$CLI_PHYSICS$CLI_CPU$CLI_ROS_WS$CLI_INSTANCE" ]; then
        # 命令行已指定部分项：不再进菜单，直接用（默认 + 覆盖）
        reselect=0
    else
        echo "  1) 使用已选默认配置（直接启动）"
        echo "  2) 重新选择"
        prompt_choice "选择 [1/2]" "1"
        case "$REPLY" in
            1|"") reselect=0 ;;
            2) reselect=1 ;;
            *)
                print_error "无效选项: $REPLY"
                exit 1
                ;;
        esac
    fi

    if [ "$reselect" -eq 1 ]; then
        echo ""
        prompt_reselect_config
        echo ""
        print_config_summary "将使用的配置"
        echo ""
        prompt_choice "写入 config/fa_sim.local.conf 作为下次默认？ [Y/n]" "Y"
        case "${REPLY,,}" in
            n|no|0) RUN_SAVE_LOCAL=0 ;;
            *) RUN_SAVE_LOCAL=1 ;;
        esac
        save_local_conf
    else
        if [ "$HEADLESS" -eq 1 ] && ! is_nonneg_int "${FA_SIM_INSTANCE}"; then
            print_error "实例序号必须是非负整数，当前: ${FA_SIM_INSTANCE}"
            exit 1
        fi
        print_info "使用已选默认配置启动"
    fi
}

source_ros_workspaces() {
    local setup underlay_setup
    # Underlay: Isaac jazzy ws（若存在且不是用户主选）
    underlay_setup="$(resolve_ros_ws_setup "${RUN_ISAAC_ROS_WS}")"
    if [ -n "$underlay_setup" ] && [ "${RUN_ROS2_WS}" != "${RUN_ISAAC_ROS_WS}" ]; then
        print_info "source Isaac ROS underlay: ${underlay_setup}"
        set +u
        # shellcheck disable=SC1090
        source "$underlay_setup"
        set -u
    fi

    setup="$(resolve_ros_ws_setup "${RUN_ROS2_WS}")"
    if [ -z "$setup" ]; then
        if [ "${RUN_ROS2_WS}" = "none" ] || [ -z "${RUN_ROS2_WS}" ]; then
            print_warn "未 source ROS2 工作空间；ROS2ControlLaunch 可能找不到 *_description"
        else
            print_error "找不到 setup.bash: ${RUN_ROS2_WS}/install/setup.bash"
            print_info "请先 colcon build，或改选正确路径"
            exit 1
        fi
        return 0
    fi
    print_info "source ROS2 工作空间: ${setup}"
    set +u
    # shellcheck disable=SC1090
    source "$setup"
    set -u
    print_info "AMENT_PREFIX_PATH 首项: ${AMENT_PREFIX_PATH%%:*}"
    print_info "ROS_DISTRO=${ROS_DISTRO:-?}  RMW=${RMW_IMPLEMENTATION:-?}"
}

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

apply_streaming_isolation() {
    local signal_port stream_port domain_id
    signal_port=$((STREAM_SIGNAL_PORT_BASE + FA_SIM_INSTANCE * STREAM_PORT_STEP))
    stream_port=$((STREAM_STREAM_PORT_BASE + FA_SIM_INSTANCE * STREAM_PORT_STEP))
    domain_id=$((ROS_DOMAIN_ID_BASE + FA_SIM_INSTANCE))

    if [ "${domain_id}" -gt 232 ]; then
        print_error "ROS_DOMAIN_ID=${domain_id} 超出有效范围 0–232"
        exit 1
    fi

    export ROS_DOMAIN_ID="${domain_id}"
    STREAM_SIGNAL_PORT="${signal_port}"
    STREAM_STREAM_PORT="${stream_port}"

    print_info "Headless 多实例隔离:"
    print_info "  FA_SIM_INSTANCE=${FA_SIM_INSTANCE}"
    print_info "  livestream signalPort(TCP)=${STREAM_SIGNAL_PORT}"
    print_info "  livestream streamPort(UDP)=${STREAM_STREAM_PORT}"
    print_info "  ROS_DOMAIN_ID=${ROS_DOMAIN_ID}"
}

build_isaac_excluded_ext_args() {
    # Carb: --/app/extensions/excluded/N=id  （覆盖 kit 内 excluded 数组）
    ISAAC_KIT_EXTRA_ARGS=()
    local i=0 ext
    for ext in "${ISAAC_EXCLUDED_EXTS[@]:-}"; do
        [ -n "$ext" ] || continue
        ISAAC_KIT_EXTRA_ARGS+=("--/app/extensions/excluded/${i}=${ext}")
        i=$((i + 1))
    done
}

sanitize_host_ros_paths_for_isaac() {
    # Full host ROS for OCS2 / ros2_control: keep /opt/ros on LD+AMENT, strip
    # only PYTHONPATH, disable pink (see ISAAC_EXCLUDED_EXTS). Vanilla
    # isaac-sim.sh still uses the aggressive strip (no FASIM_FULL_HOST_ROS).
    if [ ! -f "${SCRIPT_DIR}/tools/isaac_env_guard.sh" ]; then
        print_warn "缺少 tools/isaac_env_guard.sh，跳过 ROS 环境整理"
        return 0
    fi
    export ISAACSIM_DIR="${ISAACSIM_DIR}"
    export FASIM_FULL_HOST_ROS=1
    # shellcheck disable=SC1091
    source "${SCRIPT_DIR}/tools/isaac_env_guard.sh"
    print_info "完整 host ROS2（FASIM_FULL_HOST_ROS=1）：保留 /opt/ros LD+AMENT，仅清 PYTHONPATH"
    print_info "AMENT 首项: ${AMENT_PREFIX_PATH%%:*}"
    if [[ ":${LD_LIBRARY_PATH:-}:" != *":/opt/ros/"* ]]; then
        print_warn "LD_LIBRARY_PATH 中未见 /opt/ros；ocs2 可能缺 libtracetools.so"
    fi
    if [[ "${AMENT_PREFIX_PATH:-}" != *"/ros2_ws/"* ]] && [[ "${RUN_ROS2_WS:-}" == *ros2_ws* ]]; then
        print_warn "AMENT 中未见 ros2_ws；请确认已 source ${RUN_ROS2_WS}/install/setup.bash"
    fi
}

# --- main ---
interactive_config
set_cpu_performance
source_ros_workspaces

if [ "${RMW_IMPLEMENTATION:-}" = "rmw_zenoh_cpp" ]; then
    print_info "检测到 RMW_IMPLEMENTATION=rmw_zenoh_cpp，启动 Zenoh router..."
    setsid ros2 run rmw_zenoh_cpp rmw_zenohd &
    ZENOH_PID=$!
    print_info "Zenoh router 已在后台启动（进程组 PID: $ZENOH_PID）"
    trap cleanup_zenoh EXIT INT TERM
    sleep 1
else
    print_warn "当前未使用 rmw_zenoh_cpp（RMW_IMPLEMENTATION=${RMW_IMPLEMENTATION:-未设置}）"
fi

if [ ! -d "${ISAACSIM_DIR}" ]; then
    print_error "未找到目录: ${ISAACSIM_DIR}"
    print_info "可在 config/fa_sim.local.conf 设置 ISAACSIM_DIR"
    exit 1
fi

_ros_distro="${ROS_DISTRO:-jazzy}"
_isaac_ros2_lib="${ISAACSIM_DIR}/exts/isaacsim.ros2.core/${_ros_distro}/lib"
if [ -d "${_isaac_ros2_lib}" ]; then
    export LD_LIBRARY_PATH="${_isaac_ros2_lib}${LD_LIBRARY_PATH:+:${LD_LIBRARY_PATH}}"
    print_info "已加入 Isaac ROS2 lib: ${_isaac_ros2_lib}"
fi

sanitize_host_ros_paths_for_isaac
build_isaac_excluded_ext_args
if [ "${#ISAAC_KIT_EXTRA_ARGS[@]}" -gt 0 ]; then
    print_info "禁用冲突扩展: ${ISAAC_EXCLUDED_EXTS[*]}"
fi

cd "${ISAACSIM_DIR}" || { print_error "无法进入目录: ${ISAACSIM_DIR}"; exit 1; }

if [ "$HEADLESS" -eq 1 ]; then
    if [ "${RUN_PHYSICS_BACKEND}" = "newton" ]; then
        launch_bin="${ISAACSIM_LAUNCH_NEWTON_STREAMING:-isaac-sim.newton.streaming.sh}"
        launch_label="Headless Streaming / Newton"
    else
        launch_bin="${ISAACSIM_LAUNCH_STREAMING}"
        launch_label="Headless Streaming / PhysX"
    fi
    if [ ! -x "./${launch_bin}" ]; then
        print_error "未找到 ${ISAACSIM_DIR}/${launch_bin}"
        if [ "${RUN_PHYSICS_BACKEND}" = "newton" ]; then
            print_info "Newton headless 通常随 Isaac Sim 6.x 提供（isaac-sim.newton.streaming.sh）"
        fi
        exit 1
    fi
    apply_streaming_isolation
    print_info "启动 Isaac Sim（${launch_label}）..."
    "./${launch_bin}" \
        --/exts/omni.kit.livestream.app/primaryStream/signalPort="${STREAM_SIGNAL_PORT}" \
        --/exts/omni.kit.livestream.app/primaryStream/streamPort="${STREAM_STREAM_PORT}" \
        "${ISAAC_KIT_EXTRA_ARGS[@]}"
elif [ "${RUN_PHYSICS_BACKEND}" = "newton" ]; then
    if [ ! -x "./${ISAACSIM_LAUNCH_NEWTON}" ]; then
        print_error "未找到 ${ISAACSIM_DIR}/${ISAACSIM_LAUNCH_NEWTON}"
        print_info "Newton 通常随 Isaac Sim 6.0+ 提供"
        exit 1
    fi
    print_info "启动 Isaac Sim（Newton）..."
    "./${ISAACSIM_LAUNCH_NEWTON}" "${ISAAC_KIT_EXTRA_ARGS[@]}"
else
    if [ ! -x "./${ISAACSIM_LAUNCH_NORMAL}" ]; then
        print_error "未找到 ${ISAACSIM_DIR}/${ISAACSIM_LAUNCH_NORMAL}"
        exit 1
    fi
    print_info "启动 Isaac Sim（PhysX / 普通）..."
    "./${ISAACSIM_LAUNCH_NORMAL}" "${ISAAC_KIT_EXTRA_ARGS[@]}"
fi
