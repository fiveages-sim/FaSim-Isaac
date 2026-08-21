#!/bin/bash

# FaSim-Isaac 快速部署仓库初始化脚本
# 功能：自动初始化子模块并切换到 main 分支最新提交

set -u  # 遇到未定义变量时退出

# 颜色输出
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m'

print_info() {
    echo -e "${GREEN}[INFO]${NC} $1"
}

print_warn() {
    echo -e "${YELLOW}[WARN]${NC} $1"
}

print_error() {
    echo -e "${RED}[ERROR]${NC} $1"
}

# 获取脚本所在目录的绝对路径
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_DIR="$SCRIPT_DIR"

# 共用配置（参考 fa_w2_ws：config/*.conf，bash source）
FA_SIM_CONFIG="${REPO_DIR}/config/fa_sim.conf"
FA_SIM_LOCAL_CONFIG="${REPO_DIR}/config/fa_sim.local.conf"

load_fa_sim_config() {
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
}

load_fa_sim_config

print_info "仓库目录: $REPO_DIR"
cd "$REPO_DIR"

# 检查是否是 git 仓库
if [ ! -d ".git" ]; then
    print_error "当前目录不是 git 仓库！"
    print_info "请先克隆主仓库："
    print_info "  git clone --recurse-submodules git@github.com:fiveages-sim/FaSim-Isaac.git"
    exit 1
fi

trim() { local v="$1"; v="${v#"${v%%[![:space:]]*}"}"; echo "${v%"${v##*[![:space:]]}"}"; }

NESTED_PUBLIC_SPECS=()
NESTED_PRIVATE_SPECS=()
TOP_LEVEL_PRIVATE_PATHS=()
CATALOG_PARENT=()
CATALOG_PATH=()
CATALOG_VIS=()
CATALOG_KIND=()
CATALOG_SEL=()
CATALOG_N=0

load_visibility_conf() {
    NESTED_PUBLIC_SPECS=()
    NESTED_PRIVATE_SPECS=()
    TOP_LEVEL_PRIVATE_PATHS=()
    local line parent_dir relative_path visibility gitmodules_file spec
    while IFS= read -r line || [ -n "$line" ]; do
        line="${line%%#*}"
        line="$(trim "$line")"
        [ -z "$line" ] && continue
        IFS='|' read -r parent_dir relative_path visibility <<< "$line"
        parent_dir=$(trim "$parent_dir")
        relative_path=$(trim "$relative_path")
        visibility=$(trim "$visibility")
        gitmodules_file="${parent_dir}/.gitmodules"
        spec="${parent_dir}:${gitmodules_file}:${relative_path}"
        case "$visibility" in
            public) NESTED_PUBLIC_SPECS+=("$spec") ;;
            private)
                NESTED_PRIVATE_SPECS+=("$spec")
                if [ "$parent_dir" = "." ]; then
                    TOP_LEVEL_PRIVATE_PATHS+=("$relative_path")
                fi
                ;;
            *) print_warn "未知可见性 '$visibility'，跳过: $parent_dir/$relative_path" ;;
        esac
    done < "$VISIBILITY_CONF"
}

catalog_add() {
    local parent="$1" path="$2" vis="$3" kind="$4"
    CATALOG_PARENT+=("$parent")
    CATALOG_PATH+=("$path")
    CATALOG_VIS+=("$vis")
    CATALOG_KIND+=("$kind")
    if [ "$vis" = "public" ]; then
        CATALOG_SEL+=(1)
    else
        CATALOG_SEL+=(0)
    fi
    CATALOG_N=$((CATALOG_N + 1))
}

catalog_label() {
    local i="$1"
    if [ "${CATALOG_KIND[$i]}" = "top" ]; then
        echo "${CATALOG_PATH[$i]}"
    else
        echo "${CATALOG_PARENT[$i]}/${CATALOG_PATH[$i]}"
    fi
}

build_submodule_catalog() {
    CATALOG_PARENT=()
    CATALOG_PATH=()
    CATALOG_VIS=()
    CATALOG_KIND=()
    CATALOG_SEL=()
    CATALOG_N=0
    local p vis priv spec parent_dir rest relative_path
    local all_top_paths
    all_top_paths=$(git config --file .gitmodules --get-regexp path 2>/dev/null | awk '{print $2}')
    for p in $all_top_paths; do
        vis="public"
        for priv in "${TOP_LEVEL_PRIVATE_PATHS[@]+"${TOP_LEVEL_PRIVATE_PATHS[@]}"}"; do
            if [ "$p" = "$priv" ]; then
                vis="private"
                break
            fi
        done
        catalog_add "." "$p" "$vis" "top"
    done
    for spec in "${NESTED_PUBLIC_SPECS[@]+"${NESTED_PUBLIC_SPECS[@]}"}" "${NESTED_PRIVATE_SPECS[@]+"${NESTED_PRIVATE_SPECS[@]}"}"; do
        [ -z "${spec:-}" ] && continue
        parent_dir="${spec%%:*}"
        [ "$parent_dir" = "." ] && continue
        rest="${spec#*:}"
        relative_path="${rest#*:}"
        vis="private"
        for s in "${NESTED_PUBLIC_SPECS[@]+"${NESTED_PUBLIC_SPECS[@]}"}"; do
            [ "$s" = "$spec" ] && vis="public" && break
        done
        catalog_add "$parent_dir" "$relative_path" "$vis" "nested"
    done
}

catalog_reset_public() {
    local i
    for ((i = 0; i < CATALOG_N; i++)); do
        if [ "${CATALOG_VIS[$i]}" = "public" ]; then
            CATALOG_SEL[$i]=1
        else
            CATALOG_SEL[$i]=0
        fi
    done
}

catalog_select_all() {
    local i
    for ((i = 0; i < CATALOG_N; i++)); do
        CATALOG_SEL[$i]=1
    done
}

ensure_parents_selected() {
    local i j parent
    for ((i = 0; i < CATALOG_N; i++)); do
        if [ "${CATALOG_KIND[$i]}" = "nested" ] && [ "${CATALOG_SEL[$i]}" = "1" ]; then
            parent="${CATALOG_PARENT[$i]}"
            for ((j = 0; j < CATALOG_N; j++)); do
                if [ "${CATALOG_KIND[$j]}" = "top" ] && [ "${CATALOG_PATH[$j]}" = "$parent" ]; then
                    CATALOG_SEL[$j]=1
                fi
            done
        fi
    done
}

draw_catalog() {
    local i mark vis_tag cursor
    for ((i = 0; i < CATALOG_N; i++)); do
        if [ "${CATALOG_SEL[$i]}" = "1" ]; then
            mark="[x]"
        else
            mark="[ ]"
        fi
        vis_tag="${CATALOG_VIS[$i]}"
        if [ "$i" -eq "$1" ]; then
            cursor=">"
            printf " %s %s [%s]  %s\n" "$cursor" "$mark" "$vis_tag" "$(catalog_label "$i")"
        else
            printf "   %s [%s]  %s\n" "$mark" "$vis_tag" "$(catalog_label "$i")"
        fi
    done
}

pick_submodules_tty() {
    local idx=0 key k2 k3 total drawn=0
    local old_stty
    old_stty=$(stty -g)
    restore_picker() {
        stty "$old_stty" 2>/dev/null || true
        printf '\033[?25h'
    }
    trap restore_picker EXIT INT TERM
    stty -echo -icanon
    printf '\033[?25l'
    echo "请勾选要初始化的子模块"
    echo "  ↑/↓ 移动  空格 勾选  a 全选  n 仅 public  Enter 确认  q 取消"
    echo ""
    total=$((3 + CATALOG_N))
    draw_catalog "$idx"
    while true; do
        IFS= read -rsn1 key || key=""
        if [ "$key" = $'\x1b' ]; then
            IFS= read -rsn1 -t 0.05 k2 || k2=""
            IFS= read -rsn1 -t 0.05 k3 || k3=""
            if [ "$k3" = "A" ] || [ "$k2" = "A" ]; then
                idx=$(( (idx - 1 + CATALOG_N) % CATALOG_N ))
            elif [ "$k3" = "B" ] || [ "$k2" = "B" ]; then
                idx=$(( (idx + 1) % CATALOG_N ))
            fi
        elif [ "$key" = " " ]; then
            CATALOG_SEL[$idx]=$((1 - CATALOG_SEL[$idx]))
        elif [ "$key" = "a" ] || [ "$key" = "A" ]; then
            catalog_select_all
        elif [ "$key" = "n" ] || [ "$key" = "N" ]; then
            catalog_reset_public
        elif [ "$key" = "q" ] || [ "$key" = "Q" ]; then
            restore_picker
            trap - EXIT INT TERM
            echo ""
            echo "已取消。"
            exit 0
        elif [ -z "$key" ]; then
            restore_picker
            trap - EXIT INT TERM
            echo ""
            break
        fi
        printf '\033[%sA' "$total"
        echo "请勾选要初始化的子模块"
        echo "  ↑/↓ 移动  空格 勾选  a 全选  n 仅 public  Enter 确认  q 取消"
        echo ""
        draw_catalog "$idx"
    done
}

pick_submodules_notty() {
    local i mark vis_tag input n
    echo "请勾选要初始化的子模块（非交互终端）"
    echo "  回车=仅 public    a=全选    序号=在 public 之外额外勾选"
    echo ""
    for ((i = 0; i < CATALOG_N; i++)); do
        if [ "${CATALOG_SEL[$i]}" = "1" ]; then
            mark="[x]"
        else
            mark="[ ]"
        fi
        vis_tag="${CATALOG_VIS[$i]}"
        printf "  %2d) %s [%s]  %s\n" "$((i + 1))" "$mark" "$vis_tag" "$(catalog_label "$i")"
    done
    echo ""
    read -rp "输入序号（空格分隔）: " input
    case "$input" in
        a|A) catalog_select_all ;;
        "") ;;
        *)
            catalog_reset_public
            for n in $input; do
                if [[ "$n" =~ ^[0-9]+$ ]] && [ "$n" -ge 1 ] && [ "$n" -le "$CATALOG_N" ]; then
                    CATALOG_SEL[$((n - 1))]=1
                else
                    print_warn "忽略无效序号: $n"
                fi
            done
            ;;
    esac
}

pick_submodules() {
    if [ "$CATALOG_N" -eq 0 ]; then
        print_error "未找到任何子模块"
        exit 1
    fi
    if [ -t 0 ] && [ -t 1 ]; then
        pick_submodules_tty
    else
        pick_submodules_notty
    fi
    ensure_parents_selected
    local i
    print_info "将初始化以下子模块："
    for ((i = 0; i < CATALOG_N; i++)); do
        if [ "${CATALOG_SEL[$i]}" = "1" ]; then
            print_info "  - [$(catalog_label "$i")] (${CATALOG_VIS[$i]})"
        fi
    done
}

# 选择操作类型
echo ""
echo "请选择要执行的操作"
echo
echo "  [仓库初始化]"
echo "    1) 初始化仓库（勾选要拉取的子模块，public 默认勾选）"
echo
echo "  [环境配置]"
echo "    2) 配置 Isaac ROS2 Jazzy Workspace（下载 ROS workspaces、安装依赖并构建）"
echo
echo "  [其他]"
echo "    q) 退出"
echo
read -rp "输入选项 [1-2/q]: " choice

case "$choice" in
    1|"") INIT_MODE="repo" ;;
    2) INIT_MODE="ros2_jazzy" ;;
    q|Q)
        echo "已退出。"
        exit 0
        ;;
    *)
        print_error "无效选项: ${choice}"
        exit 1
        ;;
esac

# Isaac Sim ROS Workspace 版本选择（仅环境配置模式）
# 优先：环境变量 ISAAC_SIM_VERSION > 菜单 > ISAAC_SIM_DEFAULT_VERSION
ISAAC_SIM_VERSION="${ISAAC_SIM_VERSION:-}"
if [ "$INIT_MODE" = "ros2_jazzy" ]; then
    if [ -z "$ISAAC_SIM_VERSION" ]; then
        echo ""
        echo "请选择 Isaac Sim ROS Workspace 版本"
        echo
        echo "  [版本]"
        ver_idx=1
        for ver in "${ISAAC_SIM_VERSIONS[@]}"; do
            suffix=""
            if [ "$ver" = "$ISAAC_SIM_DEFAULT_VERSION" ]; then
                suffix="（默认）"
            fi
            echo "    ${ver_idx}) ${ver}${suffix}"
            ver_idx=$((ver_idx + 1))
        done
        custom_idx=$ver_idx
        echo "    ${custom_idx}) 自定义版本号（如 6.1.0）"
        echo
        echo "  [其他]"
        echo "    q) 退出"
        echo
        read -rp "输入选项 [1-${custom_idx}/q]（默认: ${ISAAC_SIM_DEFAULT_VERSION}）: " ver_choice
        case "${ver_choice}" in
            "" )
                ISAAC_SIM_VERSION="$ISAAC_SIM_DEFAULT_VERSION"
                ;;
            q|Q)
                echo "已退出。"
                exit 0
                ;;
            *)
                if [ "$ver_choice" = "$custom_idx" ]; then
                    read -rp "输入版本号（例如 6.1.0 或 IsaacSim-6.1.0）: " custom_ver
                    custom_ver="${custom_ver#IsaacSim-}"
                    custom_ver="${custom_ver#v}"
                    if [ -z "$custom_ver" ]; then
                        print_error "版本号不能为空"
                        exit 1
                    fi
                    ISAAC_SIM_VERSION="$custom_ver"
                elif [[ "$ver_choice" =~ ^[0-9]+$ ]] \
                    && [ "$ver_choice" -ge 1 ] \
                    && [ "$ver_choice" -lt "$custom_idx" ]; then
                    ISAAC_SIM_VERSION="${ISAAC_SIM_VERSIONS[$((ver_choice - 1))]}"
                else
                    print_error "无效选项: ${ver_choice}"
                    exit 1
                fi
                ;;
        esac
    else
        ISAAC_SIM_VERSION="${ISAAC_SIM_VERSION#IsaacSim-}"
        ISAAC_SIM_VERSION="${ISAAC_SIM_VERSION#v}"
    fi
fi

print_info "当前模式: $INIT_MODE"
if [ "$INIT_MODE" = "ros2_jazzy" ]; then
    print_info "Isaac Sim 版本: $ISAAC_SIM_VERSION"
fi
echo ""

if [ "$INIT_MODE" != "ros2_jazzy" ]; then

VISIBILITY_CONF="$REPO_DIR/submodules_visibility.conf"
if [ ! -f "$VISIBILITY_CONF" ]; then
    print_error "未找到配置文件: $VISIBILITY_CONF"
    exit 1
fi

load_visibility_conf
print_info "已从 $VISIBILITY_CONF 加载嵌套子模块配置（public: ${#NESTED_PUBLIC_SPECS[@]} 项, private: ${#NESTED_PRIVATE_SPECS[@]} 项）"
echo ""

build_submodule_catalog
pick_submodules
echo ""

print_info "同步子模块配置..."
git submodule sync

print_info "初始化选中的顶层子模块（追加，不卸载已有项）..."
top_paths_to_init=()
for ((i = 0; i < CATALOG_N; i++)); do
    if [ "${CATALOG_KIND[$i]}" = "top" ] && [ "${CATALOG_SEL[$i]}" = "1" ]; then
        top_paths_to_init+=("${CATALOG_PATH[$i]}")
    fi
done
if [ ${#top_paths_to_init[@]} -gt 0 ]; then
    git submodule update --init "${top_paths_to_init[@]}"
else
    print_warn "未勾选任何顶层子模块"
fi

# 旧版 W2 稀疏检出可能仍开着，会挡住完整 robots 工作区（例如 Gen3）
if [ -d "$REPO_DIR/robots" ] &&
   (cd "$REPO_DIR/robots" && git rev-parse --git-dir >/dev/null 2>&1) &&
   (cd "$REPO_DIR/robots" && git config --bool core.sparseCheckout 2>/dev/null | grep -qx true); then
    print_info "检测到 robots 仍启用稀疏检出（旧模式遗留），正在关闭..."
    (cd "$REPO_DIR/robots" && git sparse-checkout disable)
    print_info "✓ robots 已恢复为完整工作区"
fi

print_info "初始化选中的嵌套子模块（追加，不卸载已有项）..."
for ((i = 0; i < CATALOG_N; i++)); do
    if [ "${CATALOG_KIND[$i]}" != "nested" ] || [ "${CATALOG_SEL[$i]}" != "1" ]; then
        continue
    fi
    parent_dir="${CATALOG_PARENT[$i]}"
    relative_path="${CATALOG_PATH[$i]}"
    [ ! -d "$parent_dir" ] && continue
    (cd "$parent_dir" && git submodule sync -- "$relative_path" && git submodule update --init "$relative_path") \
        || print_warn "$parent_dir/$relative_path 初始化失败，跳过"
done

print_info "将勾选的顶层子模块切换到目标分支最新提交..."
for ((i = 0; i < CATALOG_N; i++)); do
    if [ "${CATALOG_KIND[$i]}" != "top" ] || [ "${CATALOG_SEL[$i]}" != "1" ]; then
        continue
    fi
    submodule_path="${CATALOG_PATH[$i]}"
    branch_name=$(git config --file .gitmodules --get "submodule.$submodule_path.branch" 2>/dev/null || echo "main")
    if [ ! -d "$submodule_path" ]; then
        print_warn "子模块路径不存在: $submodule_path"
        continue
    fi
    print_info "处理子模块: $submodule_path -> 分支: $branch_name"
    cd "$submodule_path"
    if ! git rev-parse --git-dir > /dev/null 2>&1; then
        print_warn "  $submodule_path 不是有效的 git 仓库，跳过"
        cd "$REPO_DIR"
        continue
    fi
    if ! git diff-index --quiet HEAD -- 2>/dev/null; then
        print_warn "  检测到本地修改，先暂存..."
        git stash push -m "Auto-stash before branch switch" || git reset --hard HEAD || true
    fi
    print_info "  获取远程更新..."
    git fetch origin || print_warn "  获取远程更新失败，继续..."
    if ! git ls-remote --exit-code --heads origin "$branch_name" > /dev/null 2>&1; then
        print_warn "  远程分支 $branch_name 不存在，跳过 $submodule_path"
        cd "$REPO_DIR"
        continue
    fi
    current_branch=$(git rev-parse --abbrev-ref HEAD 2>/dev/null || echo "HEAD")
    if [ "$current_branch" = "$branch_name" ]; then
        print_info "  已在 $branch_name 分支"
    else
        print_info "  从 $current_branch 切换到 $branch_name 分支..."
        if git show-ref --verify --quiet "refs/heads/$branch_name"; then
            git checkout "$branch_name" 2>/dev/null || git checkout -f "$branch_name" || print_error "  无法切换到 $branch_name 分支"
        else
            git checkout -b "$branch_name" "origin/$branch_name" 2>/dev/null || print_error "  无法创建/切换到 $branch_name 分支"
        fi
    fi
    print_info "  更新到最新提交..."
    git pull origin "$branch_name" || print_warn "  拉取更新失败"
    cd "$REPO_DIR"
    print_info "✓ $submodule_path 已切换到 $branch_name 分支"
done

print_info "将勾选的嵌套子模块切换到对应分支..."
for ((i = 0; i < CATALOG_N; i++)); do
    if [ "${CATALOG_KIND[$i]}" != "nested" ] || [ "${CATALOG_SEL[$i]}" != "1" ]; then
        continue
    fi
    parent_dir="${CATALOG_PARENT[$i]}"
    relative_path="${CATALOG_PATH[$i]}"
    gitmodules_file="${parent_dir}/.gitmodules"
    full_path="$REPO_DIR/$parent_dir/$relative_path"
    if [ ! -d "$full_path" ]; then continue; fi
    if ! (cd "$full_path" && git rev-parse --git-dir >/dev/null 2>&1); then continue; fi
    gf="$REPO_DIR/$gitmodules_file"
    branch_name=$(git config --file "$gf" --get "submodule.$relative_path.branch" 2>/dev/null)
    if [ -z "$branch_name" ]; then
        config_key=$(git config --file "$gf" --get-regexp 'submodule\..*\.path' 2>/dev/null | awk -v p="$relative_path" '$2==p {k=$1; gsub(/^submodule\.|\.path$/,"",k); print k; exit}')
        branch_name=$(git config --file "$gf" --get "submodule.${config_key}.branch" 2>/dev/null)
    fi
    branch_name=${branch_name:-main}
    print_info "处理嵌套子模块: $parent_dir/$relative_path -> 分支: $branch_name"
    (cd "$REPO_DIR/$parent_dir" && git submodule sync -- "$relative_path") || true
    cd "$full_path"
    git fetch origin 2>/dev/null || print_warn "  获取远程更新失败，继续..."
    if git ls-remote --exit-code --heads origin "$branch_name" >/dev/null 2>&1; then
        actual_branch="$branch_name"
    else
        actual_branch=$(git ls-remote --symref origin HEAD 2>/dev/null | awk '/^ref: refs\/heads\// {sub(/refs\/heads\//,""); print $2; exit}')
        if [ -z "$actual_branch" ]; then
            print_warn "  远程分支 $branch_name 不存在且无法获取远程默认分支，跳过"
            cd "$REPO_DIR" || exit 1
            continue
        fi
        print_warn "  远程分支 $branch_name 不存在，改用远程默认分支: $actual_branch"
    fi
    current_branch=$(git rev-parse --abbrev-ref HEAD 2>/dev/null || echo "HEAD")
    if [ "$current_branch" != "$actual_branch" ]; then
        if git show-ref --verify --quiet "refs/heads/$actual_branch"; then
            git checkout "$actual_branch" 2>/dev/null || git checkout -f "$actual_branch" 2>/dev/null || true
        else
            git checkout -b "$actual_branch" "origin/$actual_branch" 2>/dev/null || git checkout "$actual_branch" 2>/dev/null || true
        fi
    fi
    git pull origin "$actual_branch" 2>/dev/null || print_warn "  拉取更新失败"
    print_info "✓ $parent_dir/$relative_path 已切换到 $actual_branch 分支"
    cd "$REPO_DIR" || exit 1
done

echo ""
print_info "=========================================="
print_info "子模块初始化完成！"
print_info "=========================================="
echo ""
print_info "当前子模块状态："
git submodule status
echo ""
print_info "如需更新子模块到最新提交，可以运行："
print_info "  git submodule update --remote"

fi  # end of [ "$INIT_MODE" != "ros2_jazzy" ]

if [ "$INIT_MODE" = "ros2_jazzy" ]; then

    ISAAC_SIM_TAG="${ISAAC_ROS_WS_TAG_PREFIX}${ISAAC_SIM_VERSION}"
    # shellcheck disable=SC2059
    ROS_WS_ZIP_URL="$(printf "$ISAAC_ROS_WS_ZIP_URL_TEMPLATE" "$ISAAC_SIM_TAG")"
    ROS_WS_ZIP_FILE="$REPO_DIR/${ISAAC_ROS_WS_DIR_NAME}-${ISAAC_SIM_TAG}.zip"
    ROS_WS_EXTRACTED_DIR="$REPO_DIR/${ISAAC_ROS_WS_DIR_NAME}-${ISAAC_SIM_TAG}"
    ROS_WS_DIR="$REPO_DIR/${ISAAC_ROS_WS_DIR_NAME}"
    JAZZY_DST="$REPO_DIR/${ISAAC_JAZZY_WS_NAME}"
    EXISTING_VERSION=""
    if [ -f "$JAZZY_DST/.isaac_sim_version" ]; then
        EXISTING_VERSION="$(tr -d '[:space:]' < "$JAZZY_DST/.isaac_sim_version")"
    elif [ -d "$JAZZY_DST" ] && [ "$(ls -A "$JAZZY_DST" 2>/dev/null)" ]; then
        EXISTING_VERSION="unknown"
    fi

    if [ -n "$EXISTING_VERSION" ] && [ "$EXISTING_VERSION" != "$ISAAC_SIM_VERSION" ]; then
        print_info "检测到已有工作空间版本 (${EXISTING_VERSION}) 与目标 (${ISAAC_SIM_VERSION}) 不同，清除旧目录后重新配置..."
        rm -rf "$JAZZY_DST"
        if [ -d "$ROS_WS_DIR" ]; then
            rm -rf "$ROS_WS_DIR"
            print_info "✓ 已清除中间目录: $ROS_WS_DIR"
        fi
        print_info "✓ 已清除旧工作空间: $JAZZY_DST"
        EXISTING_VERSION=""
    fi

    echo ""
    print_info "=========================================="
    print_info "下载 IsaacSim ROS workspaces（${ISAAC_SIM_TAG}）..."
    print_info "=========================================="

    if [ -n "$EXISTING_VERSION" ] && [ "$EXISTING_VERSION" = "$ISAAC_SIM_VERSION" ] \
        && [ -d "$JAZZY_DST" ] && [ "$(ls -A "$JAZZY_DST" 2>/dev/null)" ]; then
        print_info "已存在匹配版本的工作空间 (${ISAAC_SIM_VERSION})，跳过下载"
    elif [ -d "$ROS_WS_DIR" ] && [ "$(ls -A "$ROS_WS_DIR" 2>/dev/null)" ]; then
        print_info "已存在目录，跳过下载: $ROS_WS_DIR"
        print_info "如需重新下载，请先手动删除该目录："
        print_info "  rm -rf \"$ROS_WS_DIR\""
    else
        print_info "开始下载: $ROS_WS_ZIP_URL"
        if wget -q --show-progress -O "$ROS_WS_ZIP_FILE" "$ROS_WS_ZIP_URL"; then
            print_info "✓ 下载完成，正在解压..."
            if unzip "$ROS_WS_ZIP_FILE" -d "$REPO_DIR"; then
                if [ -d "$ROS_WS_EXTRACTED_DIR" ]; then
                    mv "$ROS_WS_EXTRACTED_DIR" "$ROS_WS_DIR"
                else
                    # GitHub 偶发按 commit 命名解压目录，兜底匹配
                    found_dir="$(find "$REPO_DIR" -maxdepth 1 -type d -name "${ISAAC_ROS_WS_DIR_NAME}-*" | head -n 1)"
                    if [ -n "$found_dir" ] && [ -d "$found_dir" ]; then
                        mv "$found_dir" "$ROS_WS_DIR"
                    else
                        print_warn "未找到解压目录，期望: $ROS_WS_EXTRACTED_DIR"
                    fi
                fi
                rm -f "$ROS_WS_ZIP_FILE"
                print_info "✓ 解压完成: $ROS_WS_DIR"
            else
                print_warn "解压失败，请检查 zip 文件是否完整"
                rm -f "$ROS_WS_ZIP_FILE"
            fi
        else
            print_warn "下载失败。请检查网络连接，或确认版本标签是否存在："
            print_warn "  https://github.com/isaac-sim/IsaacSim-ros_workspaces/releases"
            print_warn "  wget -O \"$ROS_WS_ZIP_FILE\" \"$ROS_WS_ZIP_URL\""
            rm -f "$ROS_WS_ZIP_FILE"
        fi
    fi

    echo ""
    print_info "=========================================="
    print_info "提取 jazzy_ws 到 FaSim-Isaac 目录（${ISAAC_SIM_TAG}）..."
    print_info "=========================================="

    JAZZY_SRC="$ROS_WS_DIR/jazzy_ws"

    if [ -n "$EXISTING_VERSION" ] && [ "$EXISTING_VERSION" = "$ISAAC_SIM_VERSION" ] \
        && [ -d "$JAZZY_DST" ] && [ "$(ls -A "$JAZZY_DST" 2>/dev/null)" ]; then
        print_info "已存在匹配版本的工作空间 (${ISAAC_SIM_VERSION})，跳过提取"
    elif [ ! -d "$JAZZY_SRC" ]; then
        print_warn "未找到源目录: $JAZZY_SRC，跳过提取（可能下载失败或目录名有变化）"
    elif [ -d "$JAZZY_DST" ] && [ "$(ls -A "$JAZZY_DST" 2>/dev/null)" ]; then
        print_warn "目标目录已存在且非空，跳过提取: $JAZZY_DST"
        print_warn "如需强制重新提取，请先手动删除："
        print_warn "  rm -rf \"$JAZZY_DST\""
    else
        if mv "$JAZZY_SRC" "$JAZZY_DST"; then
            print_info "✓ 提取完成: $JAZZY_SRC -> $JAZZY_DST"
            echo "$ISAAC_SIM_VERSION" > "$JAZZY_DST/.isaac_sim_version"
        else
            print_warn "提取失败，请手动执行："
            print_warn "  mv \"$JAZZY_SRC\" \"$JAZZY_DST\""
        fi
    fi

    echo ""
    print_info "=========================================="
    print_info "安装 rosdep / colcon 依赖（apt）..."
    print_info "=========================================="

    if command -v apt >/dev/null 2>&1; then
        missing_pkgs=()
        for pkg in "${ISAAC_ROS_APT_PACKAGES[@]}"; do
            if dpkg -s "$pkg" >/dev/null 2>&1; then
                print_info "已安装: $pkg"
            else
                missing_pkgs+=("$pkg")
                print_warn "未安装: $pkg"
            fi
        done

        if [ ${#missing_pkgs[@]} -eq 0 ]; then
            print_info "所需依赖已全部安装，跳过 apt install"
        else
            print_info "将安装缺失依赖: ${missing_pkgs[*]}"
            if command -v sudo >/dev/null 2>&1; then
                echo -n "[sudo] 请输入当前用户密码以执行 apt 安装: "
                read -rs SUDO_PASS
                echo ""
                if ! echo "$SUDO_PASS" | sudo -S -v 2>/dev/null; then
                    print_warn "密码验证失败，尝试普通 sudo（可能再次弹出密码提示）"
                    sudo apt install -y "${missing_pkgs[@]}" || print_warn "apt install 失败，请检查网络/权限/软件源"
                else
                    print_info "密码验证成功，开始安装..."
                    echo "$SUDO_PASS" | sudo -S apt install -y "${missing_pkgs[@]}" || print_warn "apt install 失败，请检查网络/权限/软件源"
                fi
                unset SUDO_PASS
            else
                print_warn "未找到 sudo，无法自动安装依赖。请手动执行："
                print_warn "  apt install -y ${missing_pkgs[*]}"
            fi
        fi
    else
        print_warn "未检测到 apt（可能不是 Ubuntu/Debian）。请按你的发行版手动安装："
        print_warn "  ${ISAAC_ROS_APT_PACKAGES[*]}"
    fi

    echo ""
    print_info "=========================================="
    print_info "初始化 ${ISAAC_JAZZY_WS_NAME} 工作空间..."
    print_info "=========================================="

    SETUP_LINE="source $REPO_DIR/${ISAAC_JAZZY_WS_NAME}/install/setup.bash"

    if [ ! -d "$JAZZY_DST" ]; then
        print_warn "未找到工作空间目录: $JAZZY_DST，跳过后续步骤"
        print_warn "请确认 jazzy_ws 提取步骤已成功完成"
    elif [ -f "$JAZZY_DST/install/setup.bash" ] && grep -qF "$SETUP_LINE" "$HOME/.bashrc" 2>/dev/null; then
        print_info "环境已配置完成（install/setup.bash 存在且 ~/.bashrc 已写入），跳过构建步骤"
    else
        cd "$JAZZY_DST"

        if command -v rosdepc >/dev/null 2>&1; then
            ROSDEP_CMD="rosdepc"
            print_info "检测到 rosdepc（国内加速），优先使用"
        else
            ROSDEP_CMD="rosdep"
        fi

        if ! [ -f /etc/ros/rosdep/sources.list.d/20-default.list ]; then
            print_info "首次运行 $ROSDEP_CMD，执行 $ROSDEP_CMD init..."
            sudo "$ROSDEP_CMD" init 2>/dev/null || print_warn "$ROSDEP_CMD init 失败（可能已初始化过，忽略）"
        fi
        print_info "更新 $ROSDEP_CMD 数据库..."
        "$ROSDEP_CMD" update || print_warn "$ROSDEP_CMD update 失败，继续..."

        print_info "步骤 1/2：安装 ROS 依赖（$ROSDEP_CMD install --from-paths src --ignore-src -r -y）..."
        if "$ROSDEP_CMD" install --from-paths src --ignore-src -r -y; then
            print_info "✓ ROS 依赖安装完成"
        else
            print_warn "$ROSDEP_CMD install 失败，请检查 src/ 目录是否存在或网络是否正常"
        fi

        print_info "步骤 2/2：构建工作空间（colcon build）..."
        if colcon build; then
            print_info "✓ colcon build 完成"
        else
            print_warn "colcon build 失败，请检查构建日志：$JAZZY_DST/log/"
        fi

        if grep -qF "$SETUP_LINE" "$HOME/.bashrc" 2>/dev/null; then
            print_info "~/.bashrc 中已存在 setup.bash source 行，跳过写入"
        else
            echo "" >> "$HOME/.bashrc"
            echo "# Isaac Jazzy workspace (${ISAAC_SIM_TAG})" >> "$HOME/.bashrc"
            echo "$SETUP_LINE" >> "$HOME/.bashrc"
            print_info "✓ 已写入 ~/.bashrc: $SETUP_LINE"
            print_info "  新终端中将自动生效，当前终端请执行: source ~/.bashrc"
        fi

        cd "$REPO_DIR"
    fi

    echo ""
    print_info "=========================================="
    print_info "清理中间文件..."
    print_info "=========================================="

    if [ -f "$ROS_WS_ZIP_FILE" ]; then
        rm -f "$ROS_WS_ZIP_FILE"
        print_info "✓ 已删除 zip 包: $ROS_WS_ZIP_FILE"
    fi

    if [ -d "$ROS_WS_DIR" ]; then
        rm -rf "$ROS_WS_DIR"
        print_info "✓ 已删除解压目录: $ROS_WS_DIR"
    fi

fi

echo ""
print_info "=========================================="
print_info "全部步骤已完成！"
print_info "=========================================="
echo ""
if [ "$INIT_MODE" = "ros2_jazzy" ]; then
    print_info "Isaac ROS2 Workspace 已按 ${ISAAC_SIM_TAG} 配置完成"
    print_info "现在可以通过以下指令启动 Isaac Sim："
    echo -e "  ${GREEN}ros2 launch isaacsim run_isaacsim.launch.py${NC}"
else
    print_info "仓库初始化完成，如需更新子模块到最新提交，可以运行："
    echo -e "  ${GREEN}git submodule update --remote${NC}"
fi
echo ""
