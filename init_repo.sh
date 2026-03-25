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

# 选择初始化模式
echo ""
echo "请选择初始化模式："
echo "  1) 仅初始化 public 仓库（适用于外部用户，无需私有仓库访问权限）"
echo "  2) 初始化所有仓库，包含 private 仓库（需要内部仓库访问权限）"
read -rp "请输入选项 [1/2]（默认: 1）: " mode_choice
case "$mode_choice" in
    2) INIT_MODE="private" ;;
    *) INIT_MODE="public" ;;
esac
print_info "初始化模式: $INIT_MODE"
echo ""

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

# 初始化嵌套子模块（根据 submodules_visibility.conf）
print_info "初始化嵌套子模块（根据配置文件）..."
for spec in "${NESTED_PUBLIC_SPECS[@]}"; do
    parent_dir="${spec%%:*}"
    rest="${spec#*:}"
    relative_path="${rest#*:}"
    [ ! -d "$parent_dir" ] && continue
    (cd "$parent_dir" && git submodule update --init "$relative_path") || print_warn "$parent_dir/$relative_path 初始化失败，跳过"
done
if [ "$INIT_MODE" = "private" ]; then
    for spec in "${NESTED_PRIVATE_SPECS[@]}"; do
        parent_dir="${spec%%:*}"
        rest="${spec#*:}"
        relative_path="${rest#*:}"
        [ ! -d "$parent_dir" ] && continue
        (cd "$parent_dir" && git submodule update --init "$relative_path") || print_warn "$parent_dir/$relative_path 初始化失败，跳过"
    done
fi

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
