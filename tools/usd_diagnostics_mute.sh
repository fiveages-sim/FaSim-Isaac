#!/usr/bin/env bash
# 切换 USD 诊断静音开关（/persistent/app/usd/muteUsdDiagnostics）。
#
# 为什么需要等待：Isaac Sim 退出时会把 persistent 设置写回 user.config.json，
# 若在它运行期间修改，会被覆盖。本脚本会先等 kit 进程退出，再改文件。
#
# 用法：
#   ./tools/usd_diagnostics_mute.sh false   # 打开诊断（显示 USD 警告）← 排查用
#   ./tools/usd_diagnostics_mute.sh true    # 恢复静音
#
# 改完再启动 Isaac Sim 即可生效。
set -euo pipefail

APP_DIR="${OV_APP_DIR:-$HOME/.local/share/ov/data/Kit/Isaac-Sim Full (Newton)/6.2}"
CFG="$APP_DIR/user.config.json"
TARGET="${1:-false}"
WAIT_SECONDS="${WAIT_SECONDS:-300}"

case "$TARGET" in
  true|false) ;;
  *) echo "用法: $0 true|false" >&2; exit 1 ;;
esac

[ -f "$CFG" ] || { echo "找不到 $CFG" >&2; exit 1; }

if pgrep -f "isaacsim/kit/kit" >/dev/null 2>&1; then
  echo "Isaac Sim 仍在运行，请先退出（关闭窗口 / Ctrl+C）。"
  echo "脚本会等待退出（最多 ${WAIT_SECONDS}s）后再写配置……"
  waited=0
  while pgrep -f "isaacsim/kit/kit" >/dev/null 2>&1; do
    if [ "$waited" -ge "$WAIT_SECONDS" ]; then
      echo "等待超时，配置未修改。请退出 Isaac Sim 后重跑本脚本。" >&2
      exit 1
    fi
    sleep 2
    waited=$((waited + 2))
    [ $((waited % 20)) -eq 0 ] && echo "  …已等待 ${waited}s"
  done
  # 进程退出后给它一点时间落盘
  sleep 2
  echo "Isaac Sim 已退出。"
fi

cp -a "$CFG" "$CFG.bak.$(date +%H%M%S)"
sed -i -E "s/(\"muteUsdDiagnostics\": )(true|false)/\1${TARGET}/" "$CFG"
python3 -c "import json;json.load(open('$CFG'))" || { echo "JSON 损坏，请用备份恢复" >&2; exit 1; }

echo -n "当前设置: "
grep -o '"muteUsdDiagnostics"[^,}]*' "$CFG"
if [ "$TARGET" = "false" ]; then
  echo "现在可以启动 Isaac Sim；日志里会打印被静音的 USD 警告。"
else
  echo "已恢复静音。"
fi
