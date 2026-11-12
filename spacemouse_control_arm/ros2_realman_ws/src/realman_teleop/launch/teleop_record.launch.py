import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import yaml


def generate_launch_description():
    # 尝试找到 src 目录下的配置文件（开发模式，修改后无需重新编译）
    # 通过向上查找 ros2_realman_ws/src 目录来定位
    this_file = os.path.abspath(__file__)
    
    # 查找 workspace 根目录（包含 src, install, build 的目录）
    ws_root = this_file
    for _ in range(10):  # 最多向上查找 10 层
        ws_root = os.path.dirname(ws_root)
        if os.path.isdir(os.path.join(ws_root, "src")) and os.path.isdir(os.path.join(ws_root, "install")):
            break
    
    # 优先使用 src 目录下的配置
    src_config = os.path.join(ws_root, "src", "realman_teleop", "config", "teleop_params.yaml")
    install_config = os.path.join(get_package_share_directory("realman_teleop"), "config", "teleop_params.yaml")
    
    if os.path.exists(src_config):
        default_config = src_config
        print(f"[Launch] 使用 src 目录配置: {src_config}")
    else:
        default_config = install_config
        print(f"[Launch] 使用 install 目录配置: {install_config}")
    
    # 支持通过 launch 参数覆盖配置文件路径
    config_arg = DeclareLaunchArgument(
        "config",
        default_value=default_config,
        description="配置文件路径（修改 src 下的配置无需重新编译）"
    )
    
    config_path = default_config
    
    # 读取配置文件，动态发现相机节点
    with open(config_path, "r") as f:
        config_data = yaml.safe_load(f)
    
    # 查找所有 camera_node_* 配置块
    camera_nodes = []
    for key in config_data:
        if key.startswith("camera_node_"):
            camera_nodes.append(
                Node(
                    package="realman_teleop",
                    executable="camera_node",
                    name=key,
                    parameters=[config_path],
                )
            )
    
    # 从 dataset_recorder_node 读取 camera_names（统一配置源）
    camera_names = config_data.get("dataset_recorder_node", {}).get("ros__parameters", {}).get("camera_names", ["cam0"])
    
    # 读取 driver 的 dry_run 参数，传递给 recorder
    dry_run = config_data.get("realman_driver_node", {}).get("ros__parameters", {}).get("dry_run", False)

    return LaunchDescription(
        [
            config_arg,
            Node(
                package="realman_teleop",
                executable="encoder_node",
                name="encoder_node",
                parameters=[config_path],
            ),
            Node(
                package="realman_teleop",
                executable="teleop_node",
                name="teleop_node",
                parameters=[config_path],
            ),
            Node(
                package="realman_teleop",
                executable="realman_driver_node",
                name="realman_driver_node",
                parameters=[config_path],
            ),
            *camera_nodes,
            Node(
                package="realman_teleop",
                executable="dataset_recorder_node",
                name="dataset_recorder_node",
                parameters=[config_path, {"dry_run": dry_run}],
            ),
            Node(
                package="realman_teleop",
                executable="control_panel",
                name="control_panel_node",
                parameters=[{"camera_names": camera_names}],
            ),
        ]
    )
