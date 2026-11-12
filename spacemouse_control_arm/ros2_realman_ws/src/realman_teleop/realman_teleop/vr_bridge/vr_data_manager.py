"""
VR 数据管理器 - 统一管理三种 VR 数据源

支持的数据源:
1. ADB (USB): 通过 adb logcat 读取
2. UDP Binary (WiFi): UDP 12345 二进制数据
3. UDP JSON (兼容): UDP 6666 JSON 数据

使用方法:
    manager = VRDataManager()
    manager.start("adb")  # 或 "udp_binary" 或 "udp_json"
    
    # 获取数据
    right = manager.get_controller_data("right")
    status = manager.get_connection_status()
    
    manager.stop()
"""
import json
import socket
import threading
import time
import logging
from typing import Optional, Dict, Any
from dataclasses import dataclass

from .adb_monitor import ADBControllerMonitor, ControllerState, HeadsetState
from .binary_parser import parse_binary_vr_data, convert_binary_to_vr_format

logger = logging.getLogger(__name__)


@dataclass
class VRControllerData:
    """统一的控制器数据格式"""
    position: tuple  # (x, y, z)
    rotation: tuple  # (qx, qy, qz, qw)
    grip: float
    trigger: float
    joystick: tuple  # (x, y)
    buttons: Dict[str, int]
    timestamp: float = 0.0
    
    def to_dict(self) -> dict:
        return {
            'position': self.position,
            'rotation': self.rotation,
            'grip': self.grip,
            'trigger': self.trigger,
            'joystick': self.joystick,
            'buttons': self.buttons,
            'timestamp': self.timestamp
        }


class VRDataManager:
    """VR 数据管理器 - 统一接口"""
    
    VALID_SOURCES = ["adb", "udp_binary", "udp_json"]
    
    def __init__(self):
        self._source: Optional[str] = None
        self._running = False
        self._lock = threading.Lock()
        
        # ADB 监控器
        self._adb_monitor: Optional[ADBControllerMonitor] = None
        self._adb_device_id: Optional[str] = None
        
        # UDP 接收
        self._udp_socket: Optional[socket.socket] = None
        self._udp_thread: Optional[threading.Thread] = None
        self._udp_port: int = 12345
        
        # 数据缓存
        self._data_cache = {
            "left": None,
            "right": None,
            "headset": None,
            "timestamp": 0.0
        }
        
        # 统计
        self._recv_count = 0
        self._error_count = 0
        self._last_recv_time = 0.0
    
    def start(self, source: str, **kwargs) -> bool:
        """
        启动指定数据源
        
        Args:
            source: 数据源类型 ("adb" | "udp_binary" | "udp_json")
            **kwargs: 额外参数
                - adb_device_id: ADB 设备 ID
                - udp_port: UDP 端口 (binary=12345, json=6666)
                
        Returns:
            是否启动成功
        """
        if source not in self.VALID_SOURCES:
            logger.error(f"无效的数据源: {source}, 有效值: {self.VALID_SOURCES}")
            return False
        
        # 停止现有数据源
        if self._running:
            self.stop()
        
        self._source = source
        self._running = True
        
        if source == "adb":
            return self._start_adb(kwargs.get("adb_device_id"))
        elif source == "udp_binary":
            port = kwargs.get("udp_port", 12345)
            return self._start_udp_binary(port)
        else:  # udp_json
            port = kwargs.get("udp_port", 6666)
            return self._start_udp_json(port)
    
    def stop(self) -> None:
        """停止数据源"""
        self._running = False
        
        # 停止 ADB
        if self._adb_monitor:
            self._adb_monitor.stop()
            self._adb_monitor = None
        
        # 停止 UDP
        if self._udp_socket:
            try:
                self._udp_socket.close()
            except Exception:
                pass
            self._udp_socket = None
        
        if self._udp_thread and self._udp_thread.is_alive():
            self._udp_thread.join(timeout=2.0)
        
        self._source = None
        logger.info("VR 数据管理器已停止")
    
    def _start_adb(self, device_id: Optional[str] = None) -> bool:
        """启动 ADB 数据源"""
        try:
            self._adb_device_id = device_id
            self._adb_monitor = ADBControllerMonitor(device_id)
            
            if not self._adb_monitor.start():
                logger.error("ADB 监控启动失败")
                return False
            
            # 启动数据更新线程
            self._udp_thread = threading.Thread(target=self._adb_update_loop, daemon=True)
            self._udp_thread.start()
            
            logger.info(f"ADB 数据源已启动，设备: {self._adb_monitor.device_id}")
            return True
            
        except Exception as e:
            logger.error(f"启动 ADB 数据源失败: {e}")
            return False
    
    def _start_udp_binary(self, port: int) -> bool:
        """启动 UDP 二进制数据源"""
        try:
            self._udp_port = port
            self._udp_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            self._udp_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            self._udp_socket.bind(("0.0.0.0", port))
            self._udp_socket.settimeout(0.1)
            
            self._udp_thread = threading.Thread(target=self._udp_binary_loop, daemon=True)
            self._udp_thread.start()
            
            logger.info(f"UDP Binary 数据源已启动，端口: {port}")
            return True
            
        except Exception as e:
            logger.error(f"启动 UDP Binary 数据源失败: {e}")
            return False
    
    def _start_udp_json(self, port: int) -> bool:
        """启动 UDP JSON 数据源"""
        try:
            self._udp_port = port
            self._udp_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            self._udp_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            self._udp_socket.bind(("0.0.0.0", port))
            self._udp_socket.settimeout(0.1)
            
            self._udp_thread = threading.Thread(target=self._udp_json_loop, daemon=True)
            self._udp_thread.start()
            
            logger.info(f"UDP JSON 数据源已启动，端口: {port}")
            return True
            
        except Exception as e:
            logger.error(f"启动 UDP JSON 数据源失败: {e}")
            return False
    
    def _adb_update_loop(self) -> None:
        """ADB 数据更新循环"""
        # 记录上次数据的时间戳，用于检测是否有新数据
        last_right_ts = 0.0
        last_left_ts = 0.0
        
        while self._running and self._adb_monitor:
            try:
                # 获取右手柄数据
                right = self._adb_monitor.get_controller_data("right")
                if right and right.timestamp > last_right_ts:
                    last_right_ts = right.timestamp
                    with self._lock:
                        self._data_cache["right"] = self._convert_controller_state(right)
                        self._data_cache["timestamp"] = time.time()
                        self._last_recv_time = time.time()
                        self._recv_count += 1
                
                # 获取左手柄数据
                left = self._adb_monitor.get_controller_data("left")
                if left and left.timestamp > last_left_ts:
                    last_left_ts = left.timestamp
                    with self._lock:
                        self._data_cache["left"] = self._convert_controller_state(left)
                
                # 获取头显数据
                headset = self._adb_monitor.get_headset_data()
                if headset:
                    with self._lock:
                        self._data_cache["headset"] = headset.to_dict()
                
                time.sleep(0.01)  # 100Hz
                
            except Exception as e:
                self._error_count += 1
                if self._error_count % 100 == 1:
                    logger.warning(f"ADB 数据更新错误: {e}")
                time.sleep(0.1)
    
    def _udp_binary_loop(self) -> None:
        """UDP 二进制数据接收循环"""
        while self._running and self._udp_socket:
            try:
                data, addr = self._udp_socket.recvfrom(65535)
                binary_data = parse_binary_vr_data(data)
                
                if binary_data:
                    vr_data = convert_binary_to_vr_format(binary_data)
                    if vr_data:
                        self._update_from_vr_data(vr_data)
                        
            except socket.timeout:
                continue
            except Exception as e:
                self._error_count += 1
                if self._error_count % 100 == 1:
                    logger.warning(f"UDP Binary 接收错误: {e}")
    
    def _udp_json_loop(self) -> None:
        """UDP JSON 数据接收循环"""
        while self._running and self._udp_socket:
            try:
                data, addr = self._udp_socket.recvfrom(4096)
                
                try:
                    vr_data = json.loads(data.decode('utf-8'))
                except (json.JSONDecodeError, UnicodeDecodeError):
                    continue
                
                self._update_from_vr_data(vr_data)
                
            except socket.timeout:
                continue
            except Exception as e:
                self._error_count += 1
                if self._error_count % 100 == 1:
                    logger.warning(f"UDP JSON 接收错误: {e}")
    
    def _update_from_vr_data(self, vr_data: dict) -> None:
        """从 VR 数据字典更新缓存"""
        now = time.time()
        
        with self._lock:
            # 右控制器
            if 'right_controller' in vr_data and vr_data['right_controller']:
                rc = vr_data['right_controller']
                self._data_cache["right"] = VRControllerData(
                    position=tuple(rc.get('position', (0, 0, 0))),
                    rotation=tuple(rc.get('rotation', (0, 0, 0, 1))),
                    grip=float(rc.get('grip', 0.0)),
                    trigger=float(rc.get('trigger', 0.0)),
                    joystick=tuple(rc.get('joystick', (0, 0))),
                    buttons=rc.get('buttons', {}),
                    timestamp=now
                )
            
            # 左控制器
            if 'left_controller' in vr_data and vr_data['left_controller']:
                lc = vr_data['left_controller']
                self._data_cache["left"] = VRControllerData(
                    position=tuple(lc.get('position', (0, 0, 0))),
                    rotation=tuple(lc.get('rotation', (0, 0, 0, 1))),
                    grip=float(lc.get('grip', 0.0)),
                    trigger=float(lc.get('trigger', 0.0)),
                    joystick=tuple(lc.get('joystick', (0, 0))),
                    buttons=lc.get('buttons', {}),
                    timestamp=now
                )
            
            # 头显
            if 'headset' in vr_data and vr_data['headset']:
                self._data_cache["headset"] = vr_data['headset']
            
            self._data_cache["timestamp"] = now
            self._last_recv_time = now
            self._recv_count += 1
    
    def _convert_controller_state(self, state: ControllerState) -> VRControllerData:
        """转换 ControllerState 为统一格式"""
        return VRControllerData(
            position=state.position,
            rotation=state.rotation,
            grip=state.grip,
            trigger=state.trigger,
            joystick=state.joystick,
            buttons=state.buttons,
            timestamp=state.timestamp if state.timestamp > 0 else time.time()
        )
    
    def get_controller_data(self, side: str) -> Optional[VRControllerData]:
        """获取控制器数据"""
        with self._lock:
            return self._data_cache.get(side.lower())
    
    def get_headset_data(self) -> Optional[dict]:
        """获取头显数据"""
        with self._lock:
            return self._data_cache.get("headset")
    
    def get_all_data(self) -> dict:
        """获取所有数据"""
        with self._lock:
            return self._data_cache.copy()
    
    def get_connection_status(self) -> dict:
        """获取连接状态"""
        now = time.time()
        timeout = 0.5  # 500ms 超时
        
        with self._lock:
            is_connected = (
                self._last_recv_time > 0 and 
                now - self._last_recv_time < timeout
            )
        
        return {
            "source": self._source,
            "running": self._running,
            "connected": is_connected,
            "recv_count": self._recv_count,
            "error_count": self._error_count,
            "last_recv_time": self._last_recv_time,
            "adb_device_id": self._adb_device_id if self._source == "adb" else None,
            "udp_port": self._udp_port if self._source in ["udp_binary", "udp_json"] else None,
        }
    
    def is_connected(self) -> bool:
        """快速检查是否连接"""
        status = self.get_connection_status()
        return status["connected"]


# 测试代码
if __name__ == "__main__":
    import sys
    
    logging.basicConfig(level=logging.INFO)
    
    manager = VRDataManager()
    
    # 选择数据源
    source = sys.argv[1] if len(sys.argv) > 1 else "adb"
    print(f"使用数据源: {source}")
    
    if not manager.start(source):
        print("启动失败")
        sys.exit(1)
    
    try:
        while True:
            right = manager.get_controller_data("right")
            if right:
                print(f"右手柄: pos={right.position}, trigger={right.trigger:.2f}")
            
            status = manager.get_connection_status()
            print(f"状态: connected={status['connected']}, recv={status['recv_count']}")
            
            time.sleep(0.5)
            
    except KeyboardInterrupt:
        manager.stop()
        print("程序已退出")
