#!/usr/bin/env bash
# 用 NVIDIA 官方 .run 安装 Isaac Sim 6.2 验证过的驱动（默认 595.58.03），
# 替换 apt 安装的 615 分支。
#
# 为什么：Omniverse Technical Requirements 对 Blackwell(GeForce) 验证的是
#   570.169 / 580.95.05 / 595.58.03，并建议新 GPU 或有问题时用 Unix Driver
#   Archive 的 Production Branch + .run 安装。默认取 595 分支最新版
#   595.112.07（2026-10-20，晚于本机内核 7.0.0-42 的构建日期，更保险）；
#   如需严格对齐 Isaac 验证版本：NVIDIA_RUN_VERSION=595.58.03
#
# 用法（需要 root）：
#   sudo ./tools/install_nvidia_run.sh --dry-run    # 只打印步骤，不做任何修改
#   sudo ./tools/install_nvidia_run.sh              # 下载 + 校验 + purge apt 驱动 + 安装
#   sudo ./tools/install_nvidia_run.sh --keep-suspend-resume
#                                                   # 保留 NVreg_PreserveVideoMemoryAllocations=1
#   sudo ./tools/install_nvidia_run.sh --uninstall  # 卸载 .run 驱动（回滚第一步）
#   sudo ./tools/install_nvidia_run.sh --no-apt-pin # 不写 apt pin（默认写，防 unattended-upgrades 装回 apt 驱动）
#
# 环境变量：
#   NVIDIA_RUN_VERSION=595.58.03   目标版本（默认 595.112.07）
#   NVIDIA_RUN_CACHE=/var/tmp/nvidia-run   runfile 缓存目录
#
# 说明：全程非交互（stdin=/dev/null + --no-questions），不会等待输入，也不会因为
# 图形会话而阻塞；如果当前还有残留 NVIDIA 模块，脚本会先尽力卸载，卸不掉只告警。
# 当前无驱动加载时（nvidia-smi 不存在）可直接在桌面终端运行。
set -euo pipefail

VERSION="${NVIDIA_RUN_VERSION:-595.112.07}"
BASE="https://download.nvidia.com/XFree86/Linux-x86_64/${VERSION}"
RUNFILE="NVIDIA-Linux-x86_64-${VERSION}.run"
CACHE_DIR="${NVIDIA_RUN_CACHE:-/var/tmp/nvidia-run}"
LOG="/var/log/nvidia-run-${VERSION}.log"
MODPROBE_ISAAC="/etc/modprobe.d/zz-nvidia-isaac.conf"

DRY_RUN=0
UNINSTALL=0
KEEP_SUSPEND=0
APT_PIN=1
APT_PIN_FILE="/etc/apt/preferences.d/90-nvidia-run.pref"

# apt 管理、需要在新驱动前移除的包（不含 libnvidia-container* / nvidia-container-toolkit*）
APT_DRIVER_PKGS=(
  nvidia-driver-open nvidia-dkms-open nvidia-kernel-common nvidia-kernel-source-open
  nvidia-firmware nvidia-modprobe nvidia-persistenced nvidia-settings
  libnvidia-gl libnvidia-compute libnvidia-decode libnvidia-encode libnvidia-gpucomp
  libnvidia-cfg1 libnvidia-fbc1
  libnvidia-egl-gbm1 libnvidia-egl-wayland21 libnvidia-egl-xcb1 libnvidia-egl-xlib1
)

info()  { printf '\033[0;36m[INFO]\033[0m %s\n' "$*"; }
warn()  { printf '\033[1;33m[WARN]\033[0m %s\n' "$*"; }
error() { printf '\033[0;31m[ERROR]\033[0m %s\n' "$*" >&2; }

run() {
  if [ "$DRY_RUN" -eq 1 ]; then
    printf '  [dry-run] %s\n' "$*"
  else
    "$@"
  fi
}

usage() { awk 'NR>1 && /^set -euo pipefail/{exit} NR>1' "$0" | sed 's/^# \{0,1\}//'; }

for arg in "$@"; do
  case "$arg" in
    --dry-run) DRY_RUN=1 ;;
    --uninstall) UNINSTALL=1 ;;
    --keep-suspend-resume) KEEP_SUSPEND=1 ;;
    --no-apt-pin) APT_PIN=0 ;;
    -h|--help) usage; exit 0 ;;
    *) error "未知参数: $arg"; usage; exit 1 ;;
  esac
done

# ---------------------------------------------------------------- preflight
[ "$(id -u)" -eq 0 ] || [ "$DRY_RUN" -eq 1 ] || { error "需要 root：sudo $0 $*"; exit 1; }

echo "== NVIDIA .run 安装：${VERSION} =="
echo "  内核        : $(uname -r)"
echo "  发行版      : $(lsb_release -ds 2>/dev/null || echo '?')"
echo "  当前驱动    : $(nvidia-smi --query-gpu=driver_version --format=csv,noheader 2>/dev/null || echo '未安装/未加载')"
echo

if [ "$UNINSTALL" -eq 1 ]; then
  [ -x /usr/bin/nvidia-uninstall ] || { error "未找到 /usr/bin/nvidia-uninstall（当前不是 .run 驱动？）"; exit 1; }
  info "卸载 .run 驱动（之后可 apt install nvidia-driver-595-open 回滚到发行版包）"
  run /usr/bin/nvidia-uninstall --silent
  info "完成。建议 reboot 后再安装目标驱动。"
  exit 0
fi

if [ "$DRY_RUN" -eq 0 ]; then
  if lsmod | grep -qE '^nvidia'; then
    info "检测到残留 NVIDIA 模块，尝试卸载（失败也继续，不会阻塞）"
    for m in nvidia_drm nvidia_modeset nvidia_uvm nvidia; do
      modprobe -r "$m" 2>/dev/null || true
    done
    if lsmod | grep -qE '^nvidia'; then
      warn "仍有 NVIDIA 模块在使用中；如安装器报 sanity check 失败，重启后重跑本脚本即可"
    else
      info "  已全部卸载"
    fi
  else
    info "当前没有加载中的 NVIDIA 模块，可直接安装"
  fi
fi

# 内核头文件 / dkms / gcc
if [ "$DRY_RUN" -eq 0 ]; then
  [ -d "/lib/modules/$(uname -r)/build" ] || { error "缺少 /lib/modules/$(uname -r)/build（安装 linux-headers-$(uname -r)）"; exit 1; }
  command -v dkms >/dev/null || { error "未安装 dkms"; exit 1; }
  command -v gcc  >/dev/null || { error "未安装 gcc"; exit 1; }
fi

# ---------------------------------------------------------------- download
mkdir -p "$CACHE_DIR"
cd "$CACHE_DIR"

if [ ! -f "$RUNFILE" ]; then
  info "下载 ${RUNFILE} (~404MB，已下载可断点续传)"
  run curl -fL --retry 3 --retry-delay 2 --connect-timeout 15 --max-time 3600 \
    -C - -o "$RUNFILE" "${BASE}/${RUNFILE}"
fi

info "下载并校验 sha256"
if [ ! -f "${RUNFILE}.sha256sum" ]; then
  run curl -fL --retry 3 --connect-timeout 15 --max-time 120 \
    -o "${RUNFILE}.sha256sum" "${BASE}/${RUNFILE}.sha256sum"
fi
if [ "$DRY_RUN" -eq 0 ]; then
  if ! ( cd "$CACHE_DIR" && sha256sum -c "${RUNFILE}.sha256sum" >/dev/null 2>&1 ); then
    warn "sha256 不匹配，重新下载一次"
    rm -f "$RUNFILE"
    curl -fL --retry 3 --connect-timeout 15 --max-time 3600 -C - -o "$RUNFILE" "${BASE}/${RUNFILE}"
    ( cd "$CACHE_DIR" && sha256sum -c "${RUNFILE}.sha256sum" ) || { error "sha256 校验失败"; exit 1; }
  fi
  info "  sha256 OK"
  chmod +x "$RUNFILE"
fi

# ---------------------------------------------------------------- purge apt
export DEBIAN_FRONTEND=noninteractive
wait_dpkg_lock() {
  local n=0
  while fuser /var/lib/dpkg/lock-frontend >/dev/null 2>&1; do
    n=$((n + 1))
    if [ "$n" -gt 60 ]; then
      error "dpkg 锁被占用超过 120s（unattended-upgrades / 其它 apt 进程），请稍后重跑"
      exit 1
    fi
    [ "$n" -eq 1 ] && info "等待 dpkg 锁释放（最多 120s）..."
    sleep 2
  done
}
[ "$DRY_RUN" -eq 1 ] || wait_dpkg_lock

info "移除 apt 管理的 NVIDIA 驱动包（不触碰 libnvidia-container / CUDA toolkit）"
installed=()
for p in "${APT_DRIVER_PKGS[@]}"; do
  if dpkg-query -W -f='${Status}' "$p" 2>/dev/null | grep -q "install ok installed"; then
    installed+=("$p")
  fi
done
if [ "${#installed[@]}" -gt 0 ]; then
  info "  待卸载: ${installed[*]}"
  run apt-get -y -o Dpkg::Options::=--force-confdef -o Dpkg::Options::=--force-confold \
    purge "${installed[@]}"
else
  info "  没有需要卸载的 apt 驱动包"
fi

# 清掉可能残留的 dkms 模块
if [ "$DRY_RUN" -eq 0 ]; then
  while read -r mod ver _ _; do
    [ -n "${mod:-}" ] || continue
    case "$mod" in nvidia*) info "dkms remove ${mod}/${ver}"; dkms remove -m "$mod" -v "$ver" --all >/dev/null 2>&1 || true ;; esac
  done < <(dkms status 2>/dev/null || true)
fi

# ---------------------------------------------------------------- nouveau
if [ ! -f /etc/modprobe.d/blacklist-nouveau.conf ]; then
  info "屏蔽 nouveau"
  if [ "$DRY_RUN" -eq 1 ]; then
    printf '  [dry-run] write /etc/modprobe.d/blacklist-nouveau.conf\n'
  else
    printf 'blacklist nouveau\noptions nouveau modeset=0\n' > /etc/modprobe.d/blacklist-nouveau.conf
  fi
fi

# ---------------------------------------------------------------- install
info "运行 nvidia-installer（open kernel module + DKMS；编译模块可能需要几分钟）"
info "  日志：/var/log/nvidia-installer.log 与 ${LOG}"
INSTALL_ARGS=(
  --silent
  --no-questions
  --dkms
  --kernel-module-type=open
  --no-x-check
  --no-nouveau-check
)
if [ "$DRY_RUN" -eq 1 ]; then
  printf '  [dry-run] sh %s %s </dev/null\n' "$CACHE_DIR/$RUNFILE" "${INSTALL_ARGS[*]}"
else
  set +e
  sh "$CACHE_DIR/$RUNFILE" "${INSTALL_ARGS[@]}" </dev/null 2>&1 | tee -a "$LOG"
  rc=${PIPESTATUS[0]}
  set -e
  if [ "$rc" -ne 0 ]; then
    error "nvidia-installer 退出码 ${rc}，/var/log/nvidia-installer.log 末尾："
    tail -20 /var/log/nvidia-installer.log 2>/dev/null | sed 's/^/    /' >&2 || true
    error "若提示 nvidia-drm 仍在使用中：重启后重跑本脚本（当前无驱动，重启是安全的）"
    exit 1
  fi
fi

# ---------------------------------------------------------------- apt pin
if [ "$APT_PIN" -eq 1 ]; then
  info "写 apt pin，防止 unattended-upgrades 把 apt 驱动装回来：${APT_PIN_FILE}"
  if [ "$DRY_RUN" -eq 1 ]; then
    printf '  [dry-run] write %s\n' "$APT_PIN_FILE"
  else
    cat > "$APT_PIN_FILE" <<'EOF'
# 由 install_nvidia_run.sh 生成：NVIDIA 驱动改为官方 .run 安装，禁止 apt 再装驱动包。
# 回滚时删除本文件即可。
Package: nvidia-driver-* nvidia-dkms-* nvidia-kernel-* nvidia-firmware
Package: nvidia-modprobe nvidia-persistenced nvidia-settings
Package: libnvidia-gl libnvidia-compute libnvidia-decode libnvidia-encode
Package: libnvidia-gpucomp libnvidia-cfg1 libnvidia-fbc1 libnvidia-egl-gbm1
Package: libnvidia-egl-wayland21 libnvidia-egl-xcb1 libnvidia-egl-xlib1
Pin: release *
Pin-Priority: -1
EOF
  fi
fi

# ---------------------------------------------------------------- post
if [ "$DRY_RUN" -eq 0 ]; then
  info "刷新 initramfs"
  update-initramfs -u || warn "update-initramfs 失败，请手动执行"

  if [ "$KEEP_SUSPEND" -eq 0 ] && [ -f "$MODPROBE_ISAAC" ] && grep -q "PreserveVideoMemoryAllocations=1" "$MODPROBE_ISAAC"; then
    warn "系统内没有 nvidia-suspend/resume 服务，把 PreserveVideoMemoryAllocations 改为 0"
    sed -i 's/^options nvidia NVreg_PreserveVideoMemoryAllocations=1/options nvidia NVreg_PreserveVideoMemoryAllocations=0/' "$MODPROBE_ISAAC"
  fi

  if [ -f /lib/systemd/system/nvidia-suspend.service ]; then
    info "检测到 nvidia-suspend/resume 服务（.run 自带）"
  fi

  echo
  info "安装完成，状态如下："
  cat /proc/driver/nvidia/version 2>/dev/null || warn "模块尚未加载（重启后生效）"
  dkms status 2>/dev/null | grep -i nvidia || true
  nvidia-smi --query-gpu=name,driver_version --format=csv 2>/dev/null || true
  echo
  warn "nvidia-settings（控制面板）已被移除，如需可单独装：sudo apt install nvidia-settings"
  info "请重启：sudo reboot"
  info "回滚：sudo $0 --uninstall && sudo rm -f $APT_PIN_FILE && sudo apt install nvidia-driver-595-open"
else
  echo
  info "[dry-run] 未做任何修改。去掉 --dry-run 即真正执行。"
fi
