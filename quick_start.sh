#!/usr/bin/env bash
set -euo pipefail

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
CONTAINER_NAME="${CONTAINER_NAME:-jazzy_realman}"
ROS_WS="${ROS_WS:-${REPO_ROOT}/spacemouse_control_arm/ros2_realman_ws}"
LEFT_IFACE="${LEFT_IFACE:-enx6c1ff7eccd8e}"
RIGHT_IFACE="${RIGHT_IFACE:-enp89s0}"
LEFT_HOST_IP="${LEFT_HOST_IP:-192.168.1.100}"
RIGHT_HOST_IP="${RIGHT_HOST_IP:-192.168.1.101}"
LEFT_ARM_IP="${LEFT_ARM_IP:-192.168.1.18}"
RIGHT_ARM_IP="${RIGHT_ARM_IP:-192.168.1.19}"

case "${1:-start}" in
  start)
    if [[ "${SKIP_NETWORK_SETUP:-0}" != "1" ]]; then
      sudo ip addr flush dev "${RIGHT_IFACE}"
      sudo ip addr flush dev "${LEFT_IFACE}"
      sudo ip link set "${RIGHT_IFACE}" up
      sudo ip link set "${LEFT_IFACE}" up
      sudo ip addr add "${RIGHT_HOST_IP}/32" dev "${RIGHT_IFACE}"
      sudo ip addr add "${LEFT_HOST_IP}/32" dev "${LEFT_IFACE}"
      sudo ip route replace "${RIGHT_ARM_IP}/32" dev "${RIGHT_IFACE}" src "${RIGHT_HOST_IP}"
      sudo ip route replace "${LEFT_ARM_IP}/32" dev "${LEFT_IFACE}" src "${LEFT_HOST_IP}"
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
  -h|--help|help)
    echo "Usage: $0 [start|stop]"
    echo "Overrides: CONTAINER_NAME ROS_WS LEFT_IFACE RIGHT_IFACE SKIP_NETWORK_SETUP=1"
    ;;
  *) echo "Usage: $0 [start|stop]" >&2; exit 2 ;;
esac
