#!/usr/bin/env bash
set -euo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
case "${1:-}" in
  -h|--help)
    echo "Usage: bash setup.sh [--existing-container NAME]"
    echo "Builds the ROS workspace; does not start robot nodes."
    exit 0 ;;
  --existing-container)
    [[ $# == 2 ]] || { echo "Specify a container name." >&2; exit 2; }
    container="$2"
    workspace="${ROS_WS:-${ROOT}/spacemouse_control_arm/ros2_realman_ws}"
    docker start "${container}" >/dev/null
    ;;
  "")
    container="realman_vr_teleop"
    workspace="/workspace/spacemouse_control_arm/ros2_realman_ws"
    docker compose -f "${ROOT}/docker/compose.yaml" up -d --build
    ;;
  *) echo "Usage: bash setup.sh [--existing-container NAME]" >&2; exit 2 ;;
esac
docker exec -w "${workspace}" "${container}" bash -lc '
  set -e
  source /opt/ros/jazzy/setup.bash
  colcon build --symlink-install
  source install/setup.bash
  python3 -c "from realman_teleop.hardware import CameraCapture, RealmanDriver; from lerobot.datasets import utils; assert hasattr(utils, \"write_episode_stats\"); import web_ui.control_panel; print(\"ROS, camera, SDK, recorder and Web UI imports OK\")"
'
echo "Workspace ready in ${container}."
if [[ "${container}" == realman_vr_teleop ]]; then
  echo "Start collection: bash quick_start.sh"
else
  printf 'Start collection: CONTAINER_NAME=%q ROS_WS=%q bash quick_start.sh\n' "${container}" "${workspace}"
fi
