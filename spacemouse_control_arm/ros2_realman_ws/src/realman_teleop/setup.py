"""
Realman Teleop Package Setup

包含的子模块:
- realman_teleop: 主要 ROS2 节点
- realman_teleop.vr_bridge: VR 数据获取模块（ADB/UDP）
- web_ui: 控制面板 Web 界面（含 VR 状态显示）
"""
from glob import glob
import os

from setuptools import setup, find_packages

package_name = "realman_teleop"

# find_packages() 会自动发现所有带有 __init__.py 的包
# 包括: realman_teleop, realman_teleop.vr_bridge, web_ui

setup(
    name=package_name,
    version="0.3.0",  # 版本更新：整合 VR 状态到控制面板
    packages=find_packages(),
    data_files=[
        (
            "share/ament_index/resource_index/packages",
            ["resource/realman_teleop"],
        ),
        (f"share/{package_name}", ["package.xml"]),
        (os.path.join("share", package_name, "launch"), glob("launch/*.launch.py")),
        (os.path.join("share", package_name, "config"), glob("config/*.yaml")),
        (os.path.join("share", package_name, "web_ui", "templates"), glob("web_ui/templates/*.html")),
    ],
    install_requires=["setuptools", "flask"],
    zip_safe=True,
    maintainer="user",
    maintainer_email="user@todo.todo",
    description="ROS2 nodes for Realman teleop data collection.",
    license="MIT",
    entry_points={
        "console_scripts": [
            "encoder_node = realman_teleop.encoder_node:main",
            "teleop_node = realman_teleop.teleop_node:main",
            "realman_driver_node = realman_teleop.realman_driver_node:main",
            "camera_node = realman_teleop.camera_node:main",
            "dataset_recorder_node = realman_teleop.dataset_recorder_node:main",
            "control_panel = web_ui.control_panel:main",
            # VR模块节点
            "vr_input_node = realman_teleop.vr_input_node:main",
            "vr_teleop_node = realman_teleop.vr_teleop_node:main",
        ]
    },
)
