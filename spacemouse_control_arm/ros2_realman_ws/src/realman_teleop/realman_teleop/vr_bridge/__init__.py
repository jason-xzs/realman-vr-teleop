"""
VR Bridge 模块 - VR 数据获取统一接口

支持三种数据源：
1. ADB (USB): 通过 adb logcat 读取 VR 设备日志
2. UDP Binary (WiFi): 接收 UDP 12345 二进制数据
3. UDP JSON (兼容): 接收 UDP 6666 JSON 数据（需要 hmi-user-dev）
"""

from .adb_monitor import ADBControllerMonitor, PoseParser, ControllerState, HeadsetState
from .binary_parser import parse_binary_vr_data, convert_binary_to_vr_format
from .vr_data_manager import VRDataManager

__all__ = [
    "ADBControllerMonitor",
    "PoseParser",
    "ControllerState",
    "HeadsetState",
    "parse_binary_vr_data",
    "convert_binary_to_vr_format",
    "VRDataManager",
]
