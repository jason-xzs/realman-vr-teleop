"""
ADB 控制器监控模块 - 通过 adb logcat 读取 VR 设备数据

移植自 hmi-user-dev/web/backend/ROS_Middleware/lib/adb_monitor.py

使用方法:
    monitor = ADBControllerMonitor()
    monitor.start()
    
    # 获取数据
    right = monitor.get_controller_data("right")
    headset = monitor.get_headset_data()
    
    monitor.stop()

VR Unity App 输出的日志格式:
    VRDeviceData: RIGHT_POSE t=(x,y,z) r=(qx,qy,qz,qw) RIGHT_CONTROLLER grip=0.5 A=0 B=1 ...
    VRDeviceData: HEAD_POSE t=(x,y,z) r=(qx,qy,qz,qw) HEADSET_MOUNTED=1 IPD=0.063 EYE_HEIGHT=1.7
"""
import subprocess
import sys
import re
import threading
import logging
from dataclasses import dataclass, asdict
from typing import Tuple, Dict, Optional

logger = logging.getLogger(__name__)


@dataclass
class ControllerState:
    """单个控制器状态"""
    position: Tuple[float, float, float]
    rotation: Tuple[float, float, float, float]  # qx, qy, qz, qw
    grip: float
    trigger: float
    joystick: Tuple[float, float]
    buttons: Dict[str, int]
    timestamp: float = 0.0  # 数据更新时间戳
    
    def to_dict(self) -> dict:
        return asdict(self)


@dataclass
class HeadsetState:
    """头显状态"""
    position: Tuple[float, float, float]
    rotation: Tuple[float, float, float, float]
    mounted: int
    ipd: float
    eye_height: float
    
    def to_dict(self) -> dict:
        return asdict(self)


class PoseParser:
    """VR 日志解析器"""
    
    # 控制器数据正则表达式
    CONTROLLER_PATTERN = re.compile(
        r".*VRDeviceData:\s+"
        r"(LEFT|RIGHT)_POSE\s+"
        r"t=\(([+-]?\d*\.?\d+),\s*([+-]?\d*\.?\d+),\s*([+-]?\d*\.?\d+)\)\s+"
        r"r=\(([+-]?\d*\.?\d+),\s*([+-]?\d*\.?\d+),\s*([+-]?\d*\.?\d+),\s*([+-]?\d*\.?\d+)\)\s+"
        r"(LEFT|RIGHT)_CONTROLLER\s+"
        r"grip=([+-]?\d*\.?\d+)\s+"
        r"(?:X=(\d+)\s+Y=(\d+)|A=(\d+)\s+B=(\d+))\s+"
        r"Joy_stick_button=(\d+)\s+"
        r"trigger=([+-]?\d*\.?\d+)\s+"
        r"joystick=\(([+-]?\d*\.?\d+),\s*([+-]?\d*\.?\d+)\)"
    )
    
    # 头显数据正则表达式
    HEADSET_PATTERN = re.compile(
        r".*VRDeviceData:\s+"
        r"HEAD_POSE\s+"
        r"t=\(([+-]?\d*\.?\d+),\s*([+-]?\d*\.?\d+),\s*([+-]?\d*\.?\d+)\)\s+"
        r"r=\(([+-]?\d*\.?\d+),\s*([+-]?\d*\.?\d+),\s*([+-]?\d*\.?\d+),\s*([+-]?\d*\.?\d+)\)\s+"
        r"HEADSET_MOUNTED=(\d+)\s+IPD=([+-]?\d*\.?\d+)\s+EYE_HEIGHT=([+-]?\d*\.?\d+)"
    )
    
    @classmethod
    def parse_controller(cls, log_line: str) -> Optional[ControllerState]:
        """解析控制器数据
        
        VR Unity App 输出格式: r=(qx, qy, qz, qw)
        
        原始 hmi-user-dev 的变换:
            rotation_right = (qy, qw, qz, qx)  # 被当作 [x,y,z,w] 使用
            rotation_left = (-qy, qw, qz, qx)
        
        这意味着原始代码的实际映射是:
            scipy_x = qy_in (右) / -qy_in (左)
            scipy_y = qw_in
            scipy_z = qz_in (右) / qz_in (左)
            scipy_w = qx_in
        
        本函数输出标准 scipy [x, y, z, w] 格式，保持与原始实现相同的数学效果。
        """
        match = cls.CONTROLLER_PATTERN.search(log_line)
        if not match:
            return None
        
        try:
            controller_type = "Left" if "LEFT" in match.group(1) else "Right"
            position = tuple(map(float, match.groups()[1:4]))
            
            # 解析旋转数据: VR Unity 输出 r=(qx, qy, qz, qw)
            rotation_raw = list(map(float, match.groups()[4:8]))
            qx_in, qy_in, qz_in, qw_in = rotation_raw
            
            # 坐标变换（与 hmi-user-dev/hmi-robot-dev 保持一致）
            # 原始代码: rotation_right = (qy, qw, qz, qx) 被当作 [x,y,z,w]
            # 即: scipy_x=qy, scipy_y=qw, scipy_z=qz, scipy_w=qx
            if controller_type == "Right":
                # 右手柄: [qy, qw, qz, qx] -> scipy [x, y, z, w]
                rotation = (qy_in, qw_in, qz_in, qx_in)
            else:
                # 左手柄: [-qy, qw, qz, qx] -> scipy [x, y, z, w]
                rotation = (-qy_in, qw_in, qz_in, qx_in)
            
            # 处理按钮数据
            import time as _time
            now = _time.time()
            
            if controller_type == "Left":
                buttons = {
                    'X': int(match.group(11)),
                    'Y': int(match.group(12)),
                    'JoyStick': int(match.group(15))
                }
                return ControllerState(
                    position=position,
                    rotation=rotation,
                    grip=float(match.group(10)),
                    trigger=float(match.group(16)),
                    joystick=(float(match.group(17)), float(match.group(18))),
                    buttons=buttons,
                    timestamp=now
                )
            else:
                buttons = {
                    'A': int(match.group(13)),
                    'B': int(match.group(14)),
                    'JoyStick': int(match.group(15))
                }
                return ControllerState(
                    position=position,
                    rotation=rotation,
                    grip=float(match.group(10)),
                    trigger=float(match.group(16)),
                    joystick=(float(match.group(17)), float(match.group(18))),
                    buttons=buttons,
                    timestamp=now
                )
                
        except (ValueError, IndexError) as e:
            logger.debug(f"控制器解析错误: {e}")
            return None
    
    @classmethod
    def parse_headset(cls, log_line: str) -> Optional[HeadsetState]:
        """解析头显数据"""
        match = cls.HEADSET_PATTERN.search(log_line)
        if not match:
            return None
        
        try:
            position = tuple(map(float, match.groups()[0:3]))
            rotation = tuple(map(float, match.groups()[3:7]))
            mounted = int(match.group(8))
            ipd = float(match.group(9))
            eye_height = float(match.group(10))
            
            return HeadsetState(
                position=position,
                rotation=rotation,
                mounted=mounted,
                ipd=ipd,
                eye_height=eye_height
            )
        except (ValueError, IndexError) as e:
            logger.debug(f"头显解析错误: {e}")
            return None
    
    @classmethod
    def parse(cls, log_line: str) -> Optional[ControllerState]:
        """向后兼容方法"""
        return cls.parse_controller(log_line)


class ADBControllerMonitor:
    """ADB 控制器监控器"""
    
    def __init__(self, device_id: str = None):
        """
        初始化监控器
        
        Args:
            device_id: ADB 设备 ID，None 表示自动检测
        """
        self.device_id = device_id
        self._process = None
        self._running = False
        self._data_lock = threading.Lock()
        self._thread: Optional[threading.Thread] = None
        self.current_data = {
            "left": None,
            "right": None,
            "headset": None
        }
        
        # 统计信息
        self._recv_count = 0
        self._error_count = 0
    
    def start(self) -> bool:
        """启动监控线程"""
        if self._running:
            logger.warning("ADB 监控已在运行中")
            return True
        
        # 检查 ADB 可用性
        if not self._check_adb_available():
            logger.error("ADB 不可用，请确保已安装并在 PATH 中")
            return False
        
        self._running = True
        self._init_device()
        self._thread = threading.Thread(target=self._run_monitor, daemon=True)
        self._thread.start()
        
        logger.info(f"ADB 监控已启动，设备: {self.device_id}")
        return True
    
    def stop(self) -> None:
        """停止监控"""
        self._running = False
        if self._process:
            try:
                self._process.terminate()
                # Windows 上 terminate 可能不够，等待一小段时间后强制 kill
                import time as _time
                _time.sleep(0.1)
                if self._process.poll() is None:
                    self._process.kill()
            except Exception:
                pass
        
        if self._thread and self._thread.is_alive():
            self._thread.join(timeout=2.0)
        
        logger.info("ADB 监控已停止")
    
    def _check_adb_available(self) -> bool:
        """检查 ADB 是否可用"""
        try:
            result = subprocess.run(
                ["adb", "version"],
                capture_output=True,
                text=True,
                timeout=5
            )
            return result.returncode == 0
        except Exception as e:
            logger.error(f"检查 ADB 可用性失败: {e}")
            return False
    
    def _init_device(self) -> None:
        """初始化设备连接"""
        if not self.device_id:
            devices = self._get_connected_devices()
            if devices:
                self.device_id = devices[0]
                logger.info(f"自动选择设备: {self.device_id}")
            else:
                logger.warning("未检测到 ADB 设备")
    
    def _get_connected_devices(self) -> list:
        """获取已连接设备列表"""
        try:
            output = subprocess.check_output(["adb", "devices"], timeout=5).decode()
            devices = [
                line.split("\t")[0]
                for line in output.splitlines()
                if "\tdevice" in line
            ]
            return devices
        except Exception as e:
            logger.error(f"获取设备列表失败: {e}")
            return []
    
    def _run_monitor(self) -> None:
        """监控主循环"""
        adb_cmd = ["adb", "logcat"]
        if self.device_id:
            adb_cmd[1:1] = ["-s", self.device_id]
        
        try:
            self._process = subprocess.Popen(
                adb_cmd,
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE,
                text=False,
                bufsize=1
            )
            
            while self._running:
                line_bytes = self._process.stdout.readline()
                if not line_bytes and self._process.poll() is not None:
                    break
                
                # 安全解码
                try:
                    line = line_bytes.decode('utf-8', errors='ignore')
                except (UnicodeDecodeError, AttributeError):
                    continue
                
                if "POSE" in line:
                    self._process_line(line)
                    
        except Exception as e:
            logger.error(f"ADB 监控错误: {e}")
        finally:
            self._running = False
    
    def _process_line(self, line: str) -> None:
        """处理单行日志"""
        # 尝试解析控制器数据
        controller_data = PoseParser.parse_controller(line)
        if controller_data:
            with self._data_lock:
                if "LEFT" in line:
                    self.current_data["left"] = controller_data
                else:
                    self.current_data["right"] = controller_data
                self._recv_count += 1
            return
        
        # 尝试解析头显数据
        headset_data = PoseParser.parse_headset(line)
        if headset_data:
            with self._data_lock:
                self.current_data["headset"] = headset_data
                self._recv_count += 1
    
    def get_controller_data(self, side: str) -> Optional[ControllerState]:
        """获取指定控制器数据"""
        with self._data_lock:
            return self.current_data.get(side.lower())
    
    def get_headset_data(self) -> Optional[HeadsetState]:
        """获取头显数据"""
        with self._data_lock:
            return self.current_data.get("headset")
    
    def get_both_controllers(self) -> dict:
        """获取双手柄完整数据"""
        with self._data_lock:
            return {k: v for k, v in self.current_data.items() if k in ["left", "right"]}
    
    def get_all_data(self) -> dict:
        """获取所有数据"""
        with self._data_lock:
            return self.current_data.copy()
    
    def is_connected(self) -> bool:
        """检查是否有数据"""
        with self._data_lock:
            return any(v is not None for v in self.current_data.values())
    
    def get_stats(self) -> dict:
        """获取统计信息"""
        return {
            "running": self._running,
            "device_id": self.device_id,
            "recv_count": self._recv_count,
            "error_count": self._error_count,
        }


# 使用示例
if __name__ == "__main__":
    import time
    
    logging.basicConfig(level=logging.INFO)
    
    monitor = ADBControllerMonitor()
    
    try:
        monitor.start()
        print("等待 VR 数据...")
        
        while True:
            right = monitor.get_controller_data("right")
            if right:
                print(f"右手柄: pos={right.position}, trigger={right.trigger:.2f}")
            
            headset = monitor.get_headset_data()
            if headset:
                print(f"头显: mounted={headset.mounted}")
            
            time.sleep(0.1)
            
    except KeyboardInterrupt:
        monitor.stop()
        print("程序已退出")
