#!/usr/bin/env bash
set -euo pipefail

CONTAINER_NAME="jazzy_realman"
ROS_WS="/home/robot2/realman_teleop_v2/spacemouse_control_arm/ros2_realman_ws"
LAUNCH_FILE="realman_teleop vr_dual_teleop.launch.py"

setup_network() {
  sudo ip addr flush dev enp89s0
  sudo ip addr flush dev enx6c1ff7eccd8e

  sudo ip link set enp89s0 up
  sudo ip link set enx6c1ff7eccd8e up

  sudo ip addr add 192.168.1.101/32 dev enp89s0
  sudo ip addr add 192.168.1.100/32 dev enx6c1ff7eccd8e

  sudo ip route replace 192.168.1.19/32 dev enp89s0 src 192.168.1.101
  sudo ip route replace 192.168.1.18/32 dev enx6c1ff7eccd8e src 192.168.1.100
}

start() {
  setup_network

  echo "[quick_start] Starting ROS launch in container: ${CONTAINER_NAME}"
  echo "[quick_start] Logs will print in this terminal. Press Ctrl+C to stop."

  docker exec -it "${CONTAINER_NAME}" bash -lc "
    set -e
    cd '${ROS_WS}'
    source /opt/ros/jazzy/setup.bash
    source install/setup.bash
    exec ros2 launch ${LAUNCH_FILE}
  "
}

stop() {
  echo "[quick_start] Stopping ROS launch in container: ${CONTAINER_NAME}"
  docker exec "${CONTAINER_NAME}" bash -lc "
    pkill -INT -f 'ros2 launch ${LAUNCH_FILE}' || true
    pkill -INT -f 'vr_dual_teleop.launch.py' || true
  "
}

case "${1:-start}" in
  start)
    start
    ;;
  stop)
    stop
    ;;
  *)
    echo "Usage: $0 [start|stop]" >&2
    exit 2
    ;;
esac
