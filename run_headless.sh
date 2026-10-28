#!/bin/bash
# FaSim-Isaac Headless Streaming 入口（配置交互与 ./run.sh 相同）
exec "$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)/run.sh" --headless "$@"
