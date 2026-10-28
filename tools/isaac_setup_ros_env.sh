#!/bin/bash
# SPDX-FileCopyrightText: Copyright (c) 2025-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: Apache-2.0
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
# http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

# Script to check ROS environment and source internal libraries if needed
# FaSim: sources fasim_isaac_env_guard.sh.
#   - Vanilla isaac-sim.sh (no FASIM_FULL_HOST_ROS): strip /opt/ros so pink works.
#   - ./run.sh sets FASIM_FULL_HOST_ROS=1: keep /opt/ros LD+AMENT, disable pink.

SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
ISAAC_SIM_ROOT="$SCRIPT_DIR"

DEFAULT_ROS_DISTRO="jazzy"

# Check Ubuntu version
if [ -f /etc/os-release ]; then
    . /etc/os-release
    UBUNTU_VERSION=$(echo $VERSION_ID)

    if [[ "$UBUNTU_VERSION" == "22.04" ]]; then
        DEFAULT_ROS_DISTRO="humble"
    elif [[ "$UBUNTU_VERSION" == "24.04" ]]; then
        DEFAULT_ROS_DISTRO="jazzy"
    fi
fi

# Always know which distro Isaac bridge libs use
if [ -z "$ROS_DISTRO" ]; then
    export ROS_DISTRO="$DEFAULT_ROS_DISTRO"
fi

# Always prepend Isaac bundled ROS2 libs (even when ROS_DISTRO already set
# from bashrc — otherwise /opt/ros wins and pink's libeigenpy is shadowed).
BRIDGE_EXT_PATH="$ISAAC_SIM_ROOT/exts/isaacsim.ros2.core"
BRIDGE_LIB="$BRIDGE_EXT_PATH/$ROS_DISTRO/lib"
if [ ! -d "$BRIDGE_LIB" ]; then
    for _fasim_d in jazzy humble; do
        if [ -d "$BRIDGE_EXT_PATH/$_fasim_d/lib" ]; then
            BRIDGE_LIB="$BRIDGE_EXT_PATH/$_fasim_d/lib"
            break
        fi
    done
fi
if [ -d "$BRIDGE_LIB" ]; then
    case ":${LD_LIBRARY_PATH:-}:" in
        *":${BRIDGE_LIB}:"*) ;;
        *)
            if [ -n "${LD_LIBRARY_PATH:-}" ]; then
                export LD_LIBRARY_PATH="${BRIDGE_LIB}:${LD_LIBRARY_PATH}"
            else
                export LD_LIBRARY_PATH="${BRIDGE_LIB}"
            fi
            ;;
    esac
fi
unset _fasim_d

# Set RMW implementation to FastDDS if not already set
if [ -z "$RMW_IMPLEMENTATION" ]; then
    export RMW_IMPLEMENTATION="rmw_fastrtps_cpp"
fi

# FaSim env guard (repo tools/isaac_env_guard.sh). Prefer symlink/copy next to
# this script so vanilla ~/isaacsim/isaac-sim.sh works after init option 5.
if [ -f "${SCRIPT_DIR}/fasim_isaac_env_guard.sh" ]; then
    # shellcheck disable=SC1091
    source "${SCRIPT_DIR}/fasim_isaac_env_guard.sh"
elif [ -f "${SCRIPT_DIR}/../libraries/FaSim-Isaac/tools/isaac_env_guard.sh" ]; then
    # shellcheck disable=SC1091
    source "${SCRIPT_DIR}/../libraries/FaSim-Isaac/tools/isaac_env_guard.sh"
fi
