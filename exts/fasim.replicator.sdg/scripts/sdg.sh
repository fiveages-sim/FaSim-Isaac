#!/bin/bash
# FaSim Replicator SDG thin wrapper.
#
#   ./sdg.sh list
#   ./sdg.sh describe --robot unitree_g1d --task sinopec_test_tube
#   ./sdg.sh record   --robot unitree_g1d --task sinopec_test_tube --duration 30
#   ./sdg.sh capture  --robot unitree_g1d --task sinopec_test_tube --session <hdf5>
#
# Must run under Isaac's Python: this script re-execs via $ISAACSIM_DIR/python.sh.

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
EXT_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"
ISAACSIM_DIR="${ISAACSIM_DIR:-${HOME}/isaacsim}"
PY="${ISAACSIM_DIR}/python.sh"

if [ ! -x "${PY}" ]; then
    echo "ERROR: Isaac python not found at ${PY}" >&2
    echo "       set ISAACSIM_DIR or install Isaac Sim" >&2
    exit 1
fi

export PYTHONPATH="${EXT_ROOT}${PYTHONPATH:+:${PYTHONPATH}}"

# python.sh already sets up LD_LIBRARY_PATH / ROS underlay via setup_ros_env.sh.
exec "${PY}" -m fasim.replicator.sdg "$@"
