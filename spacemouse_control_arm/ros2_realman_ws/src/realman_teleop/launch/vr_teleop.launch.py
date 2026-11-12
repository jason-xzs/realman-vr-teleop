"""
VR 遥操作模式启动文件

启动节点：
1. realman_driver_node - 机械臂驱动（复用）
2. vr_input_node - VR 数据接收（支持 ADB/WiFi/JSON 三种数据源）
3. vr_teleop_node - VR → 关节角度映射
4. camera_node - RealSense 相机采集（多相机支持）
5. dataset_recorder_node - 数据录制节点
6. control_panel - Web UI 控制面板（VR模式）

使用方式：
    # 默认使用 USB/ADB 连接（推荐）
    ros2 launch realman_teleop vr_teleop.launch.py

    # 使用 WiFi 连接
    ros2 launch realman_teleop vr_teleop.launch.py vr_data_source:=udp_binary

    # 使用 hmi-user-dev 兼容模式
    ros2 launch realman_teleop vr_teleop.launch.py vr_data_source:=udp_json

可选参数：
    ip:=192.168.1.18            机械臂 IP（覆盖配置文件）
    dry_run:=false              仿真模式（不连接真机）
    vr_data_source:=adb         VR 数据源 (adb/udp_binary/udp_json)

注意：
- 所有参数默认从 config/teleop_params.yaml 读取
- launch 参数仅用于临时覆盖，不建议频繁使用
- 启动后访问 Web UI (http://localhost:5000) 启用VR控制
"""
import os
import yaml
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # ===== 获取配置文件路径（优先使用 src 目录，开发模式无需重新编译）=====
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
        config_file = src_config
        print(f"[Launch] 使用 src 目录配置: {src_config}")
    else:
        config_file = install_config
        print(f"[Launch] 使用 install 目录配置: {install_config}")

    # ===== 读取配置文件 =====
    with open(config_file, "r") as f:
        config_data = yaml.safe_load(f)

    # 从 dataset_recorder_node 读取 camera_names（统一配置源）
    camera_names = config_data.get("dataset_recorder_node", {}).get("ros__parameters", {}).get("camera_names", ["cam0"])

    # 读取 driver 的 dry_run 参数，传递给 recorder
    dry_run = config_data.get("realman_driver_node", {}).get("ros__parameters", {}).get("dry_run", False)

    # 读取 recorder 的 use_depth 参数，传递给 control_panel
    use_depth = config_data.get("dataset_recorder_node", {}).get("ros__parameters", {}).get("use_depth", False)

    # ===== 声明 launch 参数（仅用于临时覆盖配置文件）=====
    
    # 机械臂 IP（可选覆盖）
    ip_arg = DeclareLaunchArgument(
        "ip",
        default_value="",  # 空字符串表示使用配置文件
        description="机械臂 IP 地址（空=使用配置文件）"
    )
    
    # 仿真模式（可选覆盖）
    dry_run_arg = DeclareLaunchArgument(
        "dry_run",
        default_value="",  # 空字符串表示使用配置文件
        description="仿真模式（空=使用配置文件）"
    )
    
    # VR 数据源（可选覆盖）
    vr_data_source_arg = DeclareLaunchArgument(
        "vr_data_source",
        default_value="",  # 空字符串表示使用配置文件
        description="VR 数据源（空=使用配置文件）"
    )

    # ===== 节点定义 =====
    
    # 1. 机械臂驱动节点
    # 完全从配置文件读取参数
    driver_node = Node(
        package="realman_teleop",
        executable="realman_driver_node",
        name="realman_driver_node",
        parameters=[config_file],
        output="screen",
    )
    
    # 2. VR 输入节点
    # 完全从配置文件读取参数
    vr_input_node = Node(
        package="realman_teleop",
        executable="vr_input_node",
        name="vr_input_node",
        parameters=[config_file],
        output="screen",
    )
    
    # 3. VR 遥操作节点
    # 完全从配置文件读取参数
    vr_teleop_node = Node(
        package="realman_teleop",
        executable="vr_teleop_node",
        name="vr_teleop_node",
        parameters=[config_file],
        output="screen",
    )
    
    # 4. 相机节点（从配置文件读取）
    camera_node_cam0 = Node(
        package="realman_teleop",
        executable="camera_node",
        name="camera_node_cam0",
        parameters=[config_file],
        output="screen",
    )
    
    camera_node_cam1 = Node(
        package="realman_teleop",
        executable="camera_node",
        name="camera_node_cam1",
        parameters=[config_file],
        output="screen",
    )
    
    # 5. 控制面板（VR模式）
    control_panel_node = Node(
        package="realman_teleop",
        executable="control_panel",
        name="control_panel",
        parameters=[{
            "mode": "vr",  # VR模式
            "camera_names": camera_names,
            "use_depth": use_depth,
        }],
        output="screen",
    )

    # 6. 数据录制节点
    recorder_node = Node(
        package="realman_teleop",
        executable="dataset_recorder_node",
        name="dataset_recorder_node",
        parameters=[config_file, {"dry_run": dry_run}],
        output="screen",
    )


    return LaunchDescription([
        # 参数声明
        ip_arg,
        dry_run_arg,
        vr_data_source_arg,
        # 节点
        driver_node,
        vr_input_node,
        vr_teleop_node,
        camera_node_cam0,
        camera_node_cam1,
        control_panel_node,
        recorder_node,
    ])
