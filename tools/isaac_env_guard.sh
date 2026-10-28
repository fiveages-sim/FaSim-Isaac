#!/bin/bash
# Sourced by Isaac launchers (setup_ros_env.sh / run.sh).
#
# Two modes:
#
#   FASIM_FULL_HOST_ROS=1  (./run.sh — OCS2 / ros2_control / full workspace)
#     Keep host /opt/ros on LD_LIBRARY_PATH + AMENT (tracetools, pinocchio, …).
#     Only strip /opt/ros from PYTHONPATH so Kit Python does not import apt
#     pinocchio/eigenpy. Reorder so workspace prefixes stay before /opt/ros
#     (colcon overlay). Prepend isaacsim.ros2.core/<distro> for tinyxml2 /
#     controller_manager. Pair with disabling isaacsim.robot_motion.pink*.
#
#   default  (vanilla isaac-sim.sh — pink IK examples enabled)
#     Strip /opt/ros from PYTHONPATH, LD_LIBRARY_PATH, AMENT, COLCON, CMAKE
#     so apt eigenpy cannot shadow pink's bundled lib. Then inject ros2.core.

_fasim_strip_opt_ros_pathvar() {
    local var_name="$1"
    local old="${!var_name:-}"
    local cleaned="" entry
    [ -z "$old" ] && return 0
    local _old_ifs="$IFS"
    IFS=':'
    for entry in $old; do
        [ -z "$entry" ] && continue
        case "$entry" in
            /opt/ros/*) continue ;;
        esac
        if [ -z "$cleaned" ]; then
            cleaned="$entry"
        else
            cleaned="${cleaned}:${entry}"
        fi
    done
    IFS="$_old_ifs"
    export "$var_name=$cleaned"
}

# Move every /opt/ros/* entry to the end so workspace / Isaac prefixes win.
_fasim_opt_ros_to_end() {
    local var_name="$1"
    local old="${!var_name:-}"
    local keep="" under="" entry
    [ -z "$old" ] && return 0
    local _old_ifs="$IFS"
    IFS=':'
    for entry in $old; do
        [ -z "$entry" ] && continue
        case "$entry" in
            /opt/ros/*)
                if [ -z "$under" ]; then
                    under="$entry"
                else
                    under="${under}:${entry}"
                fi
                ;;
            *)
                if [ -z "$keep" ]; then
                    keep="$entry"
                else
                    keep="${keep}:${entry}"
                fi
                ;;
        esac
    done
    IFS="$_old_ifs"
    if [ -n "$keep" ] && [ -n "$under" ]; then
        export "$var_name=${keep}:${under}"
    elif [ -n "$under" ]; then
        export "$var_name=$under"
    else
        export "$var_name=$keep"
    fi
}

_fasim_isaac_root="${ISAACSIM_DIR:-${ISAAC_SIM_ROOT:-${HOME}/isaacsim}}"
_fasim_distro="${ROS_DISTRO:-jazzy}"
_fasim_bridge="${_fasim_isaac_root}/exts/isaacsim.ros2.core/${_fasim_distro}"
if [ ! -d "${_fasim_bridge}/share" ]; then
    for _fasim_d in jazzy humble; do
        if [ -d "${_fasim_isaac_root}/exts/isaacsim.ros2.core/${_fasim_d}/share" ]; then
            _fasim_bridge="${_fasim_isaac_root}/exts/isaacsim.ros2.core/${_fasim_d}"
            _fasim_distro="${_fasim_d}"
            break
        fi
    done
fi

if [ "${FASIM_FULL_HOST_ROS:-0}" = "1" ]; then
    # Full ROS for in-process controllers: keep LD + AMENT underlay.
    _fasim_strip_opt_ros_pathvar PYTHONPATH
    _fasim_opt_ros_to_end LD_LIBRARY_PATH
    _fasim_opt_ros_to_end AMENT_PREFIX_PATH
    _fasim_opt_ros_to_end COLCON_PREFIX_PATH
    _fasim_opt_ros_to_end CMAKE_PREFIX_PATH
    # Ensure multiarch pinocchio/coal dir is present when underlay is kept.
    if [ -d "/opt/ros/${_fasim_distro}/lib/x86_64-linux-gnu" ]; then
        case ":${LD_LIBRARY_PATH:-}:" in
            *":/opt/ros/${_fasim_distro}/lib/x86_64-linux-gnu:"*) ;;
            *)
                if [ -n "${LD_LIBRARY_PATH:-}" ]; then
                    export LD_LIBRARY_PATH="${LD_LIBRARY_PATH}:/opt/ros/${_fasim_distro}/lib/x86_64-linux-gnu"
                else
                    export LD_LIBRARY_PATH="/opt/ros/${_fasim_distro}/lib/x86_64-linux-gnu"
                fi
                ;;
        esac
    fi
    if [ -d "/opt/ros/${_fasim_distro}/lib" ]; then
        case ":${LD_LIBRARY_PATH:-}:" in
            *":/opt/ros/${_fasim_distro}/lib:"*) ;;
            *)
                if [ -n "${LD_LIBRARY_PATH:-}" ]; then
                    export LD_LIBRARY_PATH="${LD_LIBRARY_PATH}:/opt/ros/${_fasim_distro}/lib"
                else
                    export LD_LIBRARY_PATH="/opt/ros/${_fasim_distro}/lib"
                fi
                ;;
        esac
    fi
    # Re-assert overlay after appending underlay dirs.
    _fasim_opt_ros_to_end LD_LIBRARY_PATH
else
    _fasim_strip_opt_ros_pathvar PYTHONPATH
    _fasim_strip_opt_ros_pathvar LD_LIBRARY_PATH
    _fasim_strip_opt_ros_pathvar AMENT_PREFIX_PATH
    _fasim_strip_opt_ros_pathvar COLCON_PREFIX_PATH
    _fasim_strip_opt_ros_pathvar CMAKE_PREFIX_PATH
fi

if [ -d "${_fasim_bridge}/share" ]; then
    case ":${AMENT_PREFIX_PATH:-}:" in
        *":${_fasim_bridge}:"*) ;;
        *)
            if [ -n "${AMENT_PREFIX_PATH:-}" ]; then
                export AMENT_PREFIX_PATH="${_fasim_bridge}:${AMENT_PREFIX_PATH}"
            else
                export AMENT_PREFIX_PATH="${_fasim_bridge}"
            fi
            ;;
    esac
fi

if [ -d "${_fasim_bridge}/lib" ]; then
    case ":${LD_LIBRARY_PATH:-}:" in
        *":${_fasim_bridge}/lib:"*) ;;
        *)
            if [ -n "${LD_LIBRARY_PATH:-}" ]; then
                export LD_LIBRARY_PATH="${_fasim_bridge}/lib:${LD_LIBRARY_PATH}"
            else
                export LD_LIBRARY_PATH="${_fasim_bridge}/lib"
            fi
            ;;
    esac
fi

unset -f _fasim_strip_opt_ros_pathvar _fasim_opt_ros_to_end
unset _fasim_isaac_root _fasim_distro _fasim_bridge _fasim_d
