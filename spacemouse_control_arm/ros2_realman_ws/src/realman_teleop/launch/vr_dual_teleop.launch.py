"""
双臂 VR 遥操作采集启动文件。

默认读取 config/teleop_dual_params.yaml：
- 一个 vr_input_node 同时发布 /vr/left/* 和 /vr/right/*
- 两个 realman_driver_node 实例控制左右臂
- 两个 vr_teleop_node 实例分别映射左右手柄
- 多相机、单 recorder、单 Web UI
"""
import os
import json

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node


def _find_config() -> str:
    this_file = os.path.abspath(__file__)
    ws_root = this_file
    for _ in range(10):
        ws_root = os.path.dirname(ws_root)
        if os.path.isdir(os.path.join(ws_root, "src")) and os.path.isdir(os.path.join(ws_root, "install")):
            break

    src_config = os.path.join(ws_root, "src", "realman_teleop", "config", "teleop_dual_params.yaml")
    if os.path.exists(src_config):
        print(f"[Launch] 使用 src 目录双臂配置: {src_config}")
        return src_config

    install_config = os.path.join(
        get_package_share_directory("realman_teleop"), "config", "teleop_dual_params.yaml"
    )
    print(f"[Launch] 使用 install 目录双臂配置: {install_config}")
    return install_config


def generate_launch_description():
    config_file = _find_config()
    with open(config_file, "r") as f:
        raw = f.read()
    if raw.lstrip().startswith("#"):
        raw = "\n".join(line for line in raw.splitlines() if not line.lstrip().startswith("#"))
    config_data = json.loads(raw)

    recorder_params = config_data.get("dataset_recorder_node", {}).get("ros__parameters", {})
    camera_names = recorder_params.get("camera_names", ["left_wrist", "right_wrist"])
    use_depth = recorder_params.get("use_depth", False)
    arm_names = recorder_params.get("arm_names", ["left", "right"])
    dry_run = any(
        config_data.get(name, {}).get("ros__parameters", {}).get("dry_run", False)
        for name in config_data
        if name.startswith("realman_driver_")
    )

    nodes = [
        DeclareLaunchArgument(
            "config",
            default_value=config_file,
            description="双臂配置文件路径（默认 teleop_dual_params.yaml）",
        )
    ]

    for name in sorted(k for k in config_data if k.startswith("realman_driver_")):
        nodes.append(Node(
            package="realman_teleop",
            executable="realman_driver_node",
            name=name,
            parameters=[config_file],
            output="screen",
        ))

    nodes.append(Node(
        package="realman_teleop",
        executable="vr_input_node",
        name="vr_input_node",
        parameters=[config_file],
        output="screen",
    ))

    for name in sorted(k for k in config_data if k.startswith("vr_teleop_")):
        nodes.append(Node(
            package="realman_teleop",
            executable="vr_teleop_node",
            name=name,
            parameters=[config_file],
            output="screen",
        ))

    for name in sorted(k for k in config_data if k.startswith("camera_node_")):
        nodes.append(Node(
            package="realman_teleop",
            executable="camera_node",
            name=name,
            parameters=[config_file],
            output="screen",
        ))

    nodes.append(Node(
        package="realman_teleop",
        executable="dataset_recorder_node",
        name="dataset_recorder_node",
        parameters=[config_file, {"dry_run": dry_run}],
        output="screen",
    ))

    nodes.append(Node(
        package="realman_teleop",
        executable="control_panel",
        name="control_panel",
        parameters=[{
            "mode": "vr",
            "camera_names": camera_names,
            "use_depth": use_depth,
            "arm_names": arm_names,
            "driver_service_prefixes": [f"/{name}/driver" for name in arm_names],
            "driver_status_topics": [f"/{name}/driver/status" for name in arm_names],
            "driver_state_topics": recorder_params.get("state_topics", [f"/{name}/rm/state_joint_state" for name in arm_names]),
            "vr_service_prefixes": [f"/{name}/vr" for name in arm_names],
            "vr_status_topics": [f"/{name}/vr/teleop_status" for name in arm_names],
        }],
        output="screen",
    ))

    return LaunchDescription(nodes)
