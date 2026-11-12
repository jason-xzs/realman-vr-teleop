#!/usr/bin/env bash
set -euo pipefail

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
CONTAINER_NAME="${CONTAINER_NAME:-realman_vr_teleop}"
ROS_WS="${ROS_WS:-/workspace/spacemouse_control_arm/ros2_realman_ws}"

case "${1:-start}" in
  start)
    if [[ "$(docker inspect -f '{{.State.Running}}' "${CONTAINER_NAME}" 2>/dev/null || true)" != true ]]; then
      echo "Container ${CONTAINER_NAME} is not running. Run bash setup.sh, or docker start ${CONTAINER_NAME}." >&2
      exit 1
    fi
    docker exec -w "${ROS_WS}" "${CONTAINER_NAME}" test -f install/setup.bash || {
      echo "ROS workspace is not built. Run bash setup.sh first." >&2
      exit 1
    }
    if [[ "${SKIP_NETWORK_SETUP:-0}" != "1" ]]; then
      bash "${REPO_ROOT}/scripts/setup_network.sh"
    fi
    echo "Starting dual-arm VR collection in ${CONTAINER_NAME}; Ctrl+C to stop."
    docker exec -it -w "${ROS_WS}" "${CONTAINER_NAME}" bash -lc '
      set -e
      source /opt/ros/jazzy/setup.bash
      source install/setup.bash
      exec ros2 launch realman_teleop vr_dual_teleop.launch.py
    '
    ;;
  stop)
    docker exec "${CONTAINER_NAME}" bash -lc "pkill -INT -f '[v]r_dual_teleop.launch.py' || true"
    ;;
  network)
    exec bash "${REPO_ROOT}/scripts/setup_network.sh"
    ;;
  -h|--help|help)
    echo "Usage: $0 [start|stop|network]"
    echo "Overrides: CONTAINER_NAME ROS_WS LEFT_IFACE RIGHT_IFACE SKIP_NETWORK_SETUP=1"
    ;;
  *) echo "Usage: $0 [start|stop|network]" >&2; exit 2 ;;
esac
