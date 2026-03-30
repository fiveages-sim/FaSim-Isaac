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

print_info "仓库目录: $REPO_DIR"
cd "$REPO_DIR"

# 检查是否是 git 仓库
if [ ! -d ".git" ]; then
    print_error "当前目录不是 git 仓库！"
    print_info "请先克隆主仓库："
    print_info "  git clone --recurse-submodules git@github.com:fiveages-sim/FaSim-Isaac.git"
    exit 1
fi

# 选择操作类型
echo ""
echo "请选择操作类型："
echo "  1) 初始化仓库（拉取子模块并切换到目标分支）"
echo "  2) 配置环境（安装 ROS2 工作空间及依赖）"
read -rp "请输入选项 [1/2]（默认: 1）: " top_choice

case "$top_choice" in
    2)
        echo ""
        echo "请选择配置项："
        echo "  1) 配置 Isaac ROS2 Jazzy Workspace（下载 ROS workspaces、安装依赖并构建）"
        read -rp "请输入选项 [1]（默认: 1）: " env_choice
        case "$env_choice" in
            *) INIT_MODE="ros2_jazzy" ;;
        esac
        ;;
    *)
        echo ""
        echo "请选择初始化模式："
        echo "  1) 仅初始化 public 仓库（适用于外部用户，无需私有仓库访问权限）"
        echo "  2) 初始化所有仓库，包含 private 仓库（需要内部仓库访问权限）"
        echo "  3) 初始化 W2 模式（顶层全拉取，robots 仅拉取指定 private 子模块）"
        read -rp "请输入选项 [1/2/3]（默认: 1）: " repo_choice
        case "$repo_choice" in
            2) INIT_MODE="private" ;;
            3) INIT_MODE="w2" ;;
            *) INIT_MODE="public" ;;
        esac
        ;;
esac
print_info "当前模式: $INIT_MODE"
echo ""

if [ "$INIT_MODE" != "ros2_jazzy" ]; then

# 嵌套子模块可见性配置文件
VISIBILITY_CONF="$REPO_DIR/submodules_visibility.conf"
if [ ! -f "$VISIBILITY_CONF" ]; then
    print_error "未找到配置文件: $VISIBILITY_CONF"
    exit 1
fi

trim() { local v="$1"; v="${v#"${v%%[![:space:]]*}"}"; echo "${v%"${v##*[![:space:]]}"}"; }

# 从配置文件加载嵌套子模块列表（父目录 “.” 表示顶层子模块）
NESTED_PUBLIC_SPECS=()
NESTED_PRIVATE_SPECS=()
TOP_LEVEL_PRIVATE_PATHS=()
W2_TARGET_PRIVATE_KEYS=(
    "robots|manipulators/Marvin"
    "robots|humannoid/FiveAges_W2"
)
W2_SELECTED_PRIVATE_SPECS=()
while IFS= read -r line || [ -n "$line" ]; do
    line="${line%%#*}"
    line="${line#"${line%%[![:space:]]*}"}"
    line="${line%"${line##*[![:space:]]}"}"
    [ -z "$line" ] && continue
    IFS='|' read -r parent_dir relative_path visibility <<< "$line"
    parent_dir=$(trim "$parent_dir")
    relative_path=$(trim "$relative_path")
    visibility=$(trim "$visibility")
    gitmodules_file="${parent_dir}/.gitmodules"
    spec="${parent_dir}:${gitmodules_file}:${relative_path}"
    case "$visibility" in
        public)  NESTED_PUBLIC_SPECS+=("$spec") ;;
        private)
            NESTED_PRIVATE_SPECS+=("$spec")
            if [ "$parent_dir" = "." ]; then
                TOP_LEVEL_PRIVATE_PATHS+=("$relative_path")
            fi
            ;;
        *)       print_warn "未知可见性 '$visibility'，跳过: $parent_dir/$relative_path" ;;
    esac
done < "$VISIBILITY_CONF"
print_info "已从 $VISIBILITY_CONF 加载嵌套子模块配置（public: ${#NESTED_PUBLIC_SPECS[@]} 项, private: ${#NESTED_PRIVATE_SPECS[@]} 项）"
echo ""

# W2 模式下仅选择指定 private 子模块
for spec in "${NESTED_PRIVATE_SPECS[@]}"; do
    parent_dir="${spec%%:*}"
    rest="${spec#*:}"
    relative_path="${rest#*:}"
    spec_key="${parent_dir}|${relative_path}"
    for w2_key in "${W2_TARGET_PRIVATE_KEYS[@]}"; do
        if [ "$spec_key" = "$w2_key" ]; then
            W2_SELECTED_PRIVATE_SPECS+=("$spec")
            break
        fi
    done
done
if [ "$INIT_MODE" = "w2" ]; then
    print_info "W2 模式仅拉取指定 private 子模块（匹配: ${#W2_SELECTED_PRIVATE_SPECS[@]} 项）"
fi

# 同步子模块配置
print_info "同步子模块配置..."
git submodule sync

# 初始化第一层子模块（public 模式下不拉取配置为 private 的顶层子模块）
print_info "初始化子模块..."
all_top_paths=$(git config --file .gitmodules --get-regexp path 2>/dev/null | awk '{print $2}')
top_paths_to_init=()
for p in $all_top_paths; do
    if [ "$INIT_MODE" = "private" ]; then
        top_paths_to_init+=("$p")
        continue
    elif [ "$INIT_MODE" = "w2" ]; then
        # W2 模式下顶层三个子模块全部初始化
        top_paths_to_init+=("$p")
        continue
    fi
    skip=false
    for priv in "${TOP_LEVEL_PRIVATE_PATHS[@]}"; do
        if [ "$p" = "$priv" ]; then
            skip=true
            break
        fi
    done
    if [ "$skip" = false ]; then
        top_paths_to_init+=("$p")
    fi
done
if [ ${#top_paths_to_init[@]} -gt 0 ]; then
    git submodule update --init "${top_paths_to_init[@]}"
else
    print_warn ".gitmodules 中无待初始化的顶层子模块路径"
fi
if [ "$INIT_MODE" = "public" ] && [ ${#TOP_LEVEL_PRIVATE_PATHS[@]} -gt 0 ]; then
    for priv in "${TOP_LEVEL_PRIVATE_PATHS[@]}"; do
        print_info "已跳过 private 顶层子模块（可选用模式 2 拉取）: $priv"
    done
fi

# W2 模式下对 robots 做稀疏检出，只保留所需目录
if [ "$INIT_MODE" = "w2" ] && (cd "$REPO_DIR/robots" && git rev-parse --git-dir >/dev/null 2>&1); then
    print_info "W2 模式：配置 robots 稀疏检出..."
    if (
        cd "$REPO_DIR/robots" &&
        git sparse-checkout init --no-cone &&
        git sparse-checkout set --no-cone \
            "/.gitmodules" \
            "/humannoid/FiveAges_W2/" \
            "/manipulators/Marvin/" \
            "/stands/Dual_Stand1/" \
            "/sensors/" \
            "/grippers/" \
            "/dexhands/"
    ); then
        print_info "✓ robots 稀疏检出已生效（保留 W2 相关目录 + sensors/grippers/dexhands）"
    else
        print_warn "robots 稀疏检出配置失败，将按常规工作区继续"
    fi
fi

# 初始化嵌套子模块（根据 submodules_visibility.conf）
print_info "初始化嵌套子模块（根据配置文件）..."
NESTED_SPECS_TO_PROCESS=("${NESTED_PUBLIC_SPECS[@]}")
if [ "$INIT_MODE" = "private" ]; then
    NESTED_SPECS_TO_PROCESS+=("${NESTED_PRIVATE_SPECS[@]}")
elif [ "$INIT_MODE" = "w2" ]; then
    NESTED_SPECS_TO_PROCESS+=("${W2_SELECTED_PRIVATE_SPECS[@]}")
fi
for spec in "${NESTED_SPECS_TO_PROCESS[@]}"; do
    parent_dir="${spec%%:*}"
    rest="${spec#*:}"
    relative_path="${rest#*:}"
    [ ! -d "$parent_dir" ] && continue
    (cd "$parent_dir" && git submodule update --init "$relative_path") || print_warn "$parent_dir/$relative_path 初始化失败，跳过"
done

# 遍历所有第一层子模块，切换到 main 分支并拉取最新提交
print_info "将子模块切换到 main 分支最新提交..."

submodule_paths=$(git config --file .gitmodules --get-regexp path | awk '{print $2}')

for submodule_path in $submodule_paths; do
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

    # 检查本地修改，先暂存
    if ! git diff-index --quiet HEAD -- 2>/dev/null; then
        print_warn "  检测到本地修改，先暂存..."
        git stash push -m "Auto-stash before branch switch" || git reset --hard HEAD || true
    fi

    # 获取远程更新
    print_info "  获取远程更新..."
    git fetch origin || print_warn "  获取远程更新失败，继续..."

    # 检查远程分支是否存在
    if ! git ls-remote --exit-code --heads origin "$branch_name" > /dev/null 2>&1; then
        print_warn "  远程分支 $branch_name 不存在，跳过 $submodule_path"
        cd "$REPO_DIR"
        continue
    fi

    # 切换到目标分支
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

    # 拉取最新提交
    print_info "  更新到最新提交..."
    git pull origin "$branch_name" || print_warn "  拉取更新失败"

    cd "$REPO_DIR"
    print_info "✓ $submodule_path 已切换到 $branch_name 分支"
done

# 将嵌套子模块切换到对应分支并更新到最新提交
print_info "将嵌套子模块切换到对应分支..."
nested_specs=("${NESTED_PUBLIC_SPECS[@]}")
if [ "$INIT_MODE" = "private" ]; then
    nested_specs+=("${NESTED_PRIVATE_SPECS[@]}")
elif [ "$INIT_MODE" = "w2" ]; then
    nested_specs+=("${W2_SELECTED_PRIVATE_SPECS[@]}")
fi
for nested_spec in "${nested_specs[@]}"; do
    parent_dir="${nested_spec%%:*}"
    rest="${nested_spec#*:}"
    gitmodules_file="${rest%%:*}"
    relative_path="${rest#*:}"
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

    echo ""
    print_info "=========================================="
    print_info "下载 IsaacSim ROS workspaces..."
    print_info "=========================================="

    ROS_WS_ZIP_URL="https://github.com/isaac-sim/IsaacSim-ros_workspaces/archive/refs/tags/IsaacSim-5.1.0.zip"
    ROS_WS_ZIP_FILE="$REPO_DIR/IsaacSim-ros_workspaces-IsaacSim-5.1.0.zip"
    ROS_WS_EXTRACTED_DIR="$REPO_DIR/IsaacSim-ros_workspaces-IsaacSim-5.1.0"
    ROS_WS_DIR="$REPO_DIR/IsaacSim-ros_workspaces"

    if [ -d "$ROS_WS_DIR" ] && [ "$(ls -A "$ROS_WS_DIR" 2>/dev/null)" ]; then
        print_info "已存在目录，跳过下载: $ROS_WS_DIR"
        print_info "如需重新下载，请先手动删除该目录："
        print_info "  rm -rf \"$ROS_WS_DIR\""
    else
        print_info "开始下载: $ROS_WS_ZIP_URL"
        if wget -q --show-progress -O "$ROS_WS_ZIP_FILE" "$ROS_WS_ZIP_URL"; then
            print_info "✓ 下载完成，正在解压..."
            if unzip -q "$ROS_WS_ZIP_FILE" -d "$REPO_DIR"; then
                if [ -d "$ROS_WS_EXTRACTED_DIR" ]; then
                    mv "$ROS_WS_EXTRACTED_DIR" "$ROS_WS_DIR"
                fi
                rm -f "$ROS_WS_ZIP_FILE"
                print_info "✓ 解压完成: $ROS_WS_DIR"
            else
                print_warn "解压失败，请检查 zip 文件是否完整"
                rm -f "$ROS_WS_ZIP_FILE"
            fi
        else
            print_warn "下载失败。请检查网络连接或手动下载："
            print_warn "  wget -O \"$ROS_WS_ZIP_FILE\" \"$ROS_WS_ZIP_URL\""
            rm -f "$ROS_WS_ZIP_FILE"
        fi
    fi

    echo ""
    print_info "=========================================="
    print_info "提取 jazzy_ws 到 FaSim-Isaac 目录..."
    print_info "=========================================="

    ROS_WS_DIR="$REPO_DIR/IsaacSim-ros_workspaces"
    JAZZY_SRC="$ROS_WS_DIR/jazzy_ws"
    JAZZY_DST="$REPO_DIR/isaac_jazzy_ws"

    if [ ! -d "$JAZZY_SRC" ]; then
        print_warn "未找到源目录: $JAZZY_SRC，跳过提取（可能下载失败或目录名有变化）"
    elif [ -d "$JAZZY_DST" ] && [ "$(ls -A "$JAZZY_DST" 2>/dev/null)" ]; then
        print_warn "目标目录已存在且非空，跳过提取: $JAZZY_DST"
        print_warn "如需重新提取，请先手动删除或备份该目录："
        print_warn "  rm -rf \"$JAZZY_DST\""
    else
        if mv "$JAZZY_SRC" "$JAZZY_DST"; then
            print_info "✓ 提取完成: $JAZZY_SRC -> $JAZZY_DST"
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
        for pkg in python3-rosdep build-essential python3-colcon-common-extensions; do
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
        print_warn "  python3-rosdep build-essential python3-colcon-common-extensions"
    fi

    echo ""
    print_info "=========================================="
    print_info "初始化 isaac_jazzy_ws 工作空间..."
    print_info "=========================================="

    JAZZY_DST="$REPO_DIR/isaac_jazzy_ws"

    SETUP_LINE="source $REPO_DIR/isaac_jazzy_ws/install/setup.bash"

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
            echo "# Isaac Jazzy workspace" >> "$HOME/.bashrc"
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

    ROS_WS_DIR="$REPO_DIR/IsaacSim-ros_workspaces"
    ROS_WS_ZIP_FILE="$REPO_DIR/IsaacSim-ros_workspaces-IsaacSim-5.1.0.zip"

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
    print_info "现在可以通过以下指令启动 Isaac Sim："
    echo -e "  ${GREEN}ros2 launch isaacsim run_isaacsim.launch.py${NC}"
else
    print_info "仓库初始化完成，如需更新子模块到最新提交，可以运行："
    echo -e "  ${GREEN}git submodule update --remote${NC}"
fi
echo ""
