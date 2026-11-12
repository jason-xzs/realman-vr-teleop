import json
import os
import os
import socket
import threading
import time
from typing import Optional, Tuple

import numpy as np
import cv2

from .utils import add_repo_paths

os.environ.setdefault("PYNPUT_BACKEND", "dummy")
add_repo_paths()

Angler = None
_angler_import_error = None

try:
    from r3kit.devices.camera.realsense.d435 import D435
except ImportError as exc:
    try:
        from r3kit.devices.camera.realsense.d415 import D415 as D435
    except ImportError as exc2:
        raise RuntimeError("未找到 r3kit RealSense 相机模块，请检查依赖与路径。") from exc2


def _load_angler():
    global Angler, _angler_import_error
    if Angler is not None:
        return Angler
    try:
        from r3kit.devices.encoder.pdcd.angler import Angler as _Angler
    except Exception as exc:
        _angler_import_error = exc
        raise RuntimeError("未找到 r3kit Angler 编码器模块，请检查依赖、串口环境或 X11/pynput 后端。") from exc
    Angler = _Angler
    return Angler

try:
    from Robotic_Arm.rm_ctypes_wrap import rm_thread_mode_e
    from Robotic_Arm.rm_robot_interface import RoboticArm
except ImportError as exc:
    raise RuntimeError("未找到 Robotic_Arm 官方 Python API，请确认目录与依赖。") from exc


class EncoderCapture:
    """
    编码器数据采集类。
    
    注意：本类不做 EMA 滤波，只负责采集原始数据。
    EMA 滤波移至 EncoderNode 中，在 format_angle 之后进行，
    避免 0°/360° 边界跨越导致的滤波错误。
    """
    def __init__(self, port: str, fps: int, alpha: float, max_consecutive_failures: int):
        AnglerClass = _load_angler()
        self.encoder = AnglerClass(
            id=port,
            index=[1, 2, 3, 4, 5, 6, 7, 8],
            baudrate=1000000,
            fps=fps,
            gap=0.002,
            name="Angler",
        )
        self.running = False
        self.latest_data = None
        self.lock = threading.Lock()
        self.thread = threading.Thread(target=self._update, daemon=True)
        self.last_ts: Optional[float] = None
        self.inst_fps: float = 0.0
        # alpha 保留但不再使用，EMA 移至 EncoderNode
        self.alpha: float = alpha
        self.max_consecutive_failures = max_consecutive_failures
        self.consecutive_failures = 0
        self.fatal_error: Optional[str] = None

    def start(self):
        self.running = True
        self.thread.start()

    def _update(self):
        while self.running:
            try:
                data = self.encoder.get()
            except Exception as exc:
                print("[EncoderCapture] 编码器读取失败:", exc)
                with self.lock:
                    self.latest_data = None
                    self.last_ts = None
                    self.inst_fps = 0.0
                    self.consecutive_failures += 1
                    if self.max_consecutive_failures > 0 and (
                        self.consecutive_failures >= self.max_consecutive_failures
                    ):
                        self.fatal_error = (
                            f"编码器连续 {self.consecutive_failures} 次读取失败，已停止采集。"
                        )
                        self.running = False
                time.sleep(0.01)
                continue

            now = time.time()
            raw_angle = np.asarray(data["angle"], dtype=float)

            # 不再做 EMA，直接输出原始角度
            data["angle_raw"] = raw_angle
            data["angle"] = raw_angle
            data["ts"] = now
            with self.lock:
                self.consecutive_failures = 0
                self.fatal_error = None
                if self.last_ts is not None:
                    dt = now - self.last_ts
                    self.inst_fps = 1.0 / dt if dt > 0 else 0.0
                self.last_ts = now
                self.latest_data = data

    def read(self):
        with self.lock:
            return self.latest_data.copy() if self.latest_data else None

    def get_fatal_error(self) -> Optional[str]:
        with self.lock:
            return self.fatal_error

    def stop(self):
        self.running = False
        self.thread.join()


class RealmanDriver:
    """
    睿尔曼机械臂驱动（简化版）。
    
    功能说明：
    ----------
    - 使用 movej_canfd 透传接口控制关节（非阻塞）
    - 使用 SDK 控制夹爪（与单臂采集流程一致）
    - 所有控制指令都是非阻塞的，发送后立即返回
    
    设计原则：
    ----------
    - 简单直接：不使用多线程、队列、锁
    - 低延迟：API 调用后立即返回，不等待执行完成
    - 单一职责：仅负责发送控制指令，状态读取由 UDPStateReceiver 负责
    
    夹爪控制说明：
    ----------
    使用 SDK 的 rm_set_gripper_position 控制夹爪，和已验证的单臂采集流程保持一致。
    该调用约 1000ms，因此由上层驱动节点放到独立线程中执行，避免阻塞主控制循环。
    """
    
    def __init__(
        self,
        ip: str,
        port: int,
        follow: bool,
        canfd_traj_mode: int,
        canfd_radio: int,
        rm_plus_baud: int,
        dry_run: bool = False,
    ):
        """
        初始化机械臂驱动。
        
        参数：
        -----
        ip : str
            机械臂 IP 地址
        port : int
            机械臂 TCP 端口（默认 8080）
        follow : bool
            CANFD 跟随模式，True=高跟随，False=低跟随
            高跟随要求透传周期 ≤10ms，低跟随更宽松
        canfd_traj_mode : int
            CANFD 轨迹模式：0=完全透传，1=曲线拟合，2=滤波模式
        canfd_radio : int
            平滑系数（仅曲线拟合/滤波模式有效）
        rm_plus_baud : int
            RM+ 串口波特率，>0 表示开启 RM+ 协议
        dry_run : bool
            True 时不连接真实机械臂，用于测试
        """
        self.ip = ip
        self.port = port
        self.follow = follow
        self.canfd_traj_mode = canfd_traj_mode
        self.canfd_radio = canfd_radio
        self.arm_dof = 0
        self.dry_run = dry_run
        self.rm_plus_baud = rm_plus_baud  # 保存波特率，用于关闭时判断

        self.arm: Optional[RoboticArm] = None
        
        # JSON 协议专用 TCP socket（用于夹爪控制）
        self._json_sock: Optional[socket.socket] = None

        if not dry_run:
            # 创建机械臂连接（SDK 用于关节控制）
            self.arm = RoboticArm(rm_thread_mode_e.RM_TRIPLE_MODE_E)
            self.arm.rm_create_robot_arm(ip=ip, port=port)
            self.arm.rm_set_arm_run_mode(1)  # 设置为实时模式
            
            self.arm_dof = self._read_dof()
            
            # 创建 JSON 协议专用 TCP socket，并开启 RM+ 生态协议
            self._create_json_socket()
            self._enable_rm_plus_json(rm_plus_baud)

    def _create_json_socket(self) -> bool:
        """
        创建 JSON 协议专用 TCP socket。
        
        返回：
        -----
        bool
            True 创建成功，False 创建失败
        """
        try:
            self._json_sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self._json_sock.settimeout(2.0)
            self._json_sock.connect((self.ip, self.port))
            # 设置为非阻塞发送时使用的短超时
            self._json_sock.settimeout(0.1)
            print(f"[RealmanDriver] JSON 协议 socket 已连接到 {self.ip}:{self.port}")
            return True
        except Exception as exc:
            print(f"[RealmanDriver] JSON 协议 socket 连接失败: {exc}")
            self._json_sock = None
            return False

    def _send_json_command(self, cmd_dict: dict, wait_response: bool = True) -> Optional[dict]:
        """
        通过 JSON 协议发送命令。
        
        参数：
        -----
        cmd_dict : dict
            命令字典
        wait_response : bool
            是否等待响应，False 时仅发送不接收（用于高频控制）
        
        返回：
        -----
        Optional[dict]
            响应字典（wait_response=True 时），或 None
        """
        if self._json_sock is None:
            return None
        
        cmd = json.dumps(cmd_dict) + "\r\n"
        try:
            self._json_sock.send(cmd.encode('utf-8'))
            
            if not wait_response:
                return {"success": True}
            
            # 等待响应
            self._json_sock.settimeout(2.0)
            data = self._json_sock.recv(4096)
            self._json_sock.settimeout(0.1)
            
            response_str = data.decode('utf-8').strip()
            # 如果响应包含多行，只取第一行
            if '\n' in response_str:
                response_str = response_str.split('\n')[0]
            return json.loads(response_str)
        except socket.timeout:
            return None
        except Exception as exc:
            print(f"[RealmanDriver] JSON 命令发送失败: {exc}")
            return None

    def _enable_rm_plus_json(self, baud: int) -> bool:
        """
        使用 JSON 协议开启 RM+ 生态协议。
        
        注意：必须使用 JSON 协议开启，SDK 方式虽然返回成功但夹爪控制仍走老 modbus 协议。
        
        参数：
        -----
        baud : int
            RM+ 串口波特率（0=禁用, 9600/115200/256000/460800）
        
        返回：
        -----
        bool
            True 开启成功，False 开启失败
        """
        if self.dry_run or int(baud) <= 0:
            return False
        
        cmd = {"command": "set_rm_plus_mode", "mode": int(baud)}
        resp = self._send_json_command(cmd, wait_response=True)
        
        if resp and resp.get("set_state") is True:
            print(f"[RealmanDriver] RM+ 生态协议已开启 (波特率={baud})")
            return True
        else:
            print(f"[RealmanDriver] RM+ 生态协议开启失败，响应: {resp}")
            return False

    def _disable_rm_plus_json(self) -> bool:
        """
        使用 JSON 协议关闭 RM+ 生态协议。
        
        重要：程序退出前必须调用，否则可能导致末端生态崩溃！
        
        返回：
        -----
        bool
            True 关闭成功，False 关闭失败
        """
        if self.dry_run:
            return True
        
        # 如果 socket 已关闭，尝试重新建立连接
        if self._json_sock is None:
            self._create_json_socket()
        
        if self._json_sock is None:
            print("[RealmanDriver] 无法关闭 RM+ 生态协议：socket 不可用")
            return False
        
        cmd = {"command": "set_rm_plus_mode", "mode": 0}
        resp = self._send_json_command(cmd, wait_response=True)
        
        if resp and resp.get("set_state") is True:
            print("[RealmanDriver] RM+ 生态协议已关闭")
            return True
        else:
            print(f"[RealmanDriver] RM+ 生态协议关闭失败，响应: {resp}")
            return False

    def _read_dof(self) -> int:
        """读取机械臂自由度。"""
        code, info = self.arm.rm_get_robot_info()
        dof = info.arm_dof if hasattr(info, "arm_dof") else 0
        if dof == 0:
            _, joint_deg = self.arm.rm_get_joint_degree()
            dof = len(joint_deg)
        return dof

    def send_joint_deg(self, joint_deg: np.ndarray) -> Optional[int]:
        """
        发送关节角度指令（非阻塞）。
        
        使用 CANFD 透传接口，指令发送后立即返回，不等待执行完成。
        
        参数：
        -----
        joint_deg : np.ndarray
            目标关节角度（单位：度）
        
        返回：
        -----
        Optional[int]
            0=成功，1=参数错误，-1=发送失败，None=dry_run 模式
        """
        if self.dry_run:
            return 0
        target = list(joint_deg[: self.arm_dof or len(joint_deg)])
        return self.arm.rm_movej_canfd(
            target, self.follow, 0.0, self.canfd_traj_mode, self.canfd_radio
        )

    def send_pose_canfd(self, pose: list) -> Optional[int]:
        """
        发送笛卡尔位姿指令（非阻塞）。
        
        使用 CANFD 透传接口直接发送位姿，由机械臂控制器内部进行 IK 反解。
        适用于 VR 遥操作等需要直接控制末端位姿的场景。
        
        参数：
        -----
        pose : list
            目标位姿，支持两种格式：
            - 7 维：[x, y, z, qw, qx, qy, qz] - 位置（米）+ 四元数
            - 6 维：[x, y, z, rx, ry, rz] - 位置（米）+ 欧拉角（弧度）
        
        返回：
        -----
        Optional[int]
            0=成功，1=参数错误，-1=发送失败，None=dry_run 模式
        """
        if self.dry_run:
            return 0
        
        if self.arm is None:
            print("[RealmanDriver] 机械臂未连接，无法发送位姿指令")
            return -1
        
        if len(pose) not in (6, 7):
            print(f"[RealmanDriver] 位姿维度错误: {len(pose)}，应为 6 或 7")
            return 1
        
        return self.arm.rm_movep_canfd(
            pose, self.follow, self.canfd_traj_mode, self.canfd_radio
        )

    def set_follow_mode(self, follow: bool) -> None:
        """
        动态切换 CANFD 跟随模式。

        参数：
        -----
        follow : bool
            True=高跟随模式（要求透传周期≤10ms，延迟更低）
            False=低跟随模式（更稳定，容错更大）
        """
        old_mode = self.follow
        self.follow = follow
        mode_str = "高跟随" if follow else "低跟随"
        print(f"[RealmanDriver] CANFD 模式已切换: {old_mode} → {follow} ({mode_str})")

    def move_to_home(self, home_joint_deg: list, velocity: int = 20,
                      open_gripper: bool = True) -> tuple:
        """
        移动到 home 位姿（阻塞式）。

        使用 SDK 的 rm_movej 阻塞模式，等待机械臂到达目标位置后返回。
        默认 home 完成后会张开夹爪（香蕉抓取任务的常见需求），但 hex 插入
        任务等需要保持夹紧六角块的场景应传 open_gripper=False。

        参数：
        -----
        home_joint_deg : list
            目标关节角度（单位：度），7 维列表
        velocity : int
            运动速度 (1-100)，默认 20
        open_gripper : bool
            home 完成后是否自动张开夹爪。默认 True（向后兼容）。
            hex 任务由 driver_node 根据 _gripper_lock_closed 传 False。

        返回：
        -----
        tuple[bool, str]
            (成功标志, 消息)
        """
        if self.dry_run:
            time.sleep(1.0)  # 仿真模式模拟运动时间
            return True, "仿真模式: 模拟回 home 完成"

        if self.arm is None:
            return False, "机械臂未连接"

        try:
            # 使用阻塞式 movej，block=1 表示等待运动完成
            # rm_movej(joint, v, r, connect, block)
            # joint: 目标角度(度), v: 速度1-100, r: 交融半径, connect: 轨迹连接, block: 阻塞
            result = self.arm.rm_movej(home_joint_deg, velocity, 0, 0, 1)

            if result == 0:
                if open_gripper:
                    # 运动成功，同时控制夹爪张开
                    self.send_gripper(1000)  # 1000 = 完全张开
                    return True, "机械臂已到达 home 位置，夹爪已张开"
                else:
                    return True, "机械臂已到达 home 位置（夹爪状态保持不变）"
            else:
                return False, f"movej 运动失败，错误码: {result}"
        except Exception as e:
            return False, f"移动到 home 位姿异常: {e}"

    def send_gripper(self, position: int) -> Optional[int]:
        """发送夹爪位置指令，position 范围 0~1000。"""
        if self.dry_run:
            return 0
        if self.arm is None:
            return -1
        try:
            # RM_ARM+ / tool modbus 与标准夹爪协议互斥。发送前切回工具端夹爪模式，
            # 避免手动关闭 RM_ARM+ 后某一侧仍停在 RTU/生态协议模式而吞掉夹爪命令。
            mode_ret = self.arm.rm_set_tool_rs485_mode(2, 115200)
            if mode_ret not in (0, -4):
                print(f"[RealmanDriver] 设置工具端夹爪模式返回: {mode_ret}")
            sdk_position = int(max(1, min(1000, int(position))))
            result = self.arm.rm_set_gripper_position(sdk_position, False, 0)
            print(
                "[RealmanDriver] SDK夹爪发送: "
                f"request={position}, sdk_position={sdk_position}, "
                f"mode_ret={mode_ret}, result={result}"
            )
            return result
        except Exception as exc:
            print(f"[RealmanDriver] 夹爪命令发送失败: {exc}")
            return -1

    def close(self):
        """
        关闭机械臂连接。
        
        重要：会先关闭 RM+ 生态协议，防止末端生态崩溃！
        """
        if self.dry_run:
            return
        
        # 1. 先关闭 RM+ 生态协议（重要！防止末端生态崩溃）
        if self.rm_plus_baud > 0:
            print("[RealmanDriver] 正在关闭 RM+ 生态协议...")
            self._disable_rm_plus_json()
        
        # 2. 关闭 JSON 协议 socket
        if self._json_sock is not None:
            try:
                self._json_sock.close()
            except Exception:
                pass
            self._json_sock = None
        
        # 3. 关闭 SDK 连接
        if self.arm is not None:
            try:
                self.arm.rm_delete_robot_arm()
            except Exception as exc:
                print("[RealmanDriver] 断开 SDK 连接失败:", exc)

    def stop_motion(self):
        """紧急停止机械臂。"""
        if self.arm is None or self.dry_run:
            return
        try:
            self.arm.rm_set_arm_emergency_stop(True)
        except Exception as exc:
            print("[RealmanDriver] 紧急停止失败:", exc)

    def enable_udp_realtime_push(
        self,
        target_ip: str,
        target_port: int = 8089,
        cycle: int = 1,
        enable: bool = True,
    ) -> bool:
        """
        配置机械臂 UDP 状态主动上报功能。
        
        功能说明：
        ----------
        向机械臂发送 set_realtime_push 命令，开启/关闭 UDP 状态主动上报。
        开启后，机械臂会以指定周期向目标 IP:端口 发送状态数据。
        
        参数：
        -----
        target_ip : str
            上报目标 IP 地址（运行本程序的主机 IP）
        target_port : int
            上报目标端口，默认 8089
        cycle : int
            上报周期，单位为 5ms 的倍数。
            例如：cycle=1 表示 5ms (200Hz)，cycle=2 表示 10ms (100Hz)
        enable : bool
            True 开启上报，False 关闭上报
        
        返回：
        -----
        bool
            True 配置成功，False 配置失败
        
        注意事项：
        ----------
        - target_ip 必须是机械臂能访问到的 IP（通常是同一局域网）
        - 配置会立即生效，机械臂开始/停止向指定地址发送 UDP 数据
        - 参考文档: https://develop.realman-robotics.com/robot4th/json/udpConfig/
        """
        if self.arm is None or self.dry_run:
            print("[RealmanDriver] dry_run 模式或未连接，跳过 UDP 配置")
            return False
        
        # 构造 set_realtime_push 命令
        # custom 字段指定需要上报的数据项
        cmd = {
            "command": "set_realtime_push",
            "enable": enable,
            "port": target_port,
            "cycle": cycle,
            "ip": target_ip,
            "force_coordinate": 0,  # 0=传感器坐标系
            "custom": {
                "joint_speed": True,           # 关节速度
                "arm_current_status": True,    # 机械臂当前状态
                "rm_plus_state": True,         # RM+ 末端设备状态（含夹爪）
                "rm_plus_base": True,          # RM+ 末端设备基础信息
            },
        }
        cmd_json = json.dumps(cmd)
        
        try:
            # 通过机械臂 API 发送 JSON 命令
            # rm_send_command 是睿尔曼 SDK 提供的通用 JSON 命令发送接口
            result = self.arm.rm_send_command(cmd_json)
            
            if result == 0:
                status = "开启" if enable else "关闭"
                print(f"[RealmanDriver] UDP 上报已{status}，目标 {target_ip}:{target_port}，周期 {cycle * 5}ms")
                return True
            else:
                print(f"[RealmanDriver] UDP 配置失败，返回码: {result}")
                return False
        except AttributeError:
            # 如果 SDK 没有 rm_send_command 方法，尝试直接 TCP 发送
            return self._send_json_command_direct(cmd_json, target_ip, target_port, enable, cycle)
        except Exception as exc:
            print(f"[RealmanDriver] UDP 配置异常: {exc}")
            return False
    
    def _send_json_command_direct(
        self,
        cmd_json: str,
        target_ip: str,
        target_port: int,
        enable: bool,
        cycle: int,
    ) -> bool:
        """
        直接通过 TCP 发送 JSON 命令（备用方法）。
        
        当 SDK 不支持 rm_send_command 时，直接建立 TCP 连接发送命令。
        """
        try:
            # 使用初始化时保存的机械臂 IP 和端口
            arm_ip = self.ip
            arm_port = self.port
            
            sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            sock.settimeout(2.0)
            sock.connect((arm_ip, arm_port))
            sock.sendall(cmd_json.encode("utf-8"))
            
            # 接收响应
            response = sock.recv(1024).decode("utf-8")
            sock.close()
            
            # 解析响应
            resp_json = json.loads(response)
            if resp_json.get("state") is True or resp_json.get("set_realtime_push") is True:
                status = "开启" if enable else "关闭"
                print(f"[RealmanDriver] UDP 上报已{status}（直接TCP），目标 {target_ip}:{target_port}，周期 {cycle * 5}ms")
                return True
            else:
                print(f"[RealmanDriver] UDP 配置失败，响应: {response}")
                return False
        except Exception as exc:
            print(f"[RealmanDriver] 直接 TCP 发送失败: {exc}")
            return False


class CameraCapture:
    def __init__(
        self,
        serial: Optional[str],
        use_depth: bool,
        name: str,
        convert_bgr_to_rgb: bool,
        flush_frames: int,
    ):
        self.camera = D435(id=serial, depth=use_depth, name=name)
        self.use_depth = use_depth
        self.convert_bgr_to_rgb = convert_bgr_to_rgb
        self.flush_frames = max(0, int(flush_frames))

    def get(self):
        for _ in range(self.flush_frames):
            self.camera.get()
        color, depth = self.camera.get()
        if self.convert_bgr_to_rgb:
            color = cv2.cvtColor(color, cv2.COLOR_BGR2RGB)
        if not self.use_depth:
            depth = None
        return color, depth


class UDPStateReceiver:
    """
    睿尔曼机械臂 UDP 状态接收器。
    
    功能说明：
    ----------
    通过 UDP 协议接收机械臂主动上报的状态数据，相比 TCP 请求-响应模式：
    - 不阻塞：独立 UDP 端口，与控制指令完全分离
    - 高频率：最高支持 200Hz (5ms 周期)
    - 低延迟：数据主动推送，无需等待请求响应
    
    使用方法：
    ----------
    1. 创建实例时指定监听端口（默认 8089）
    2. 调用 start() 启动后台接收线程
    3. 调用 read_joint_deg() 获取最新关节角度
    4. 调用 read_gripper_pos() 获取最新夹爪位置
    5. 程序结束前调用 stop() 停止接收
    
    注意事项：
    ----------
    - 需要先通过 TCP 向机械臂发送 set_realtime_push 命令开启 UDP 上报
    - 上报目标 IP 需设置为运行本程序的主机 IP
    - 默认端口 8089，可通过参数修改
    
    数据格式参考：
    ----------
    睿尔曼官方文档: https://develop.realman-robotics.com/robot4th/json/udpConfig/
    """
    
    def __init__(self, port: int = 8089, arm_dof: int = 7):
        """
        初始化 UDP 状态接收器。
        
        参数：
        -----
        port : int
            UDP 监听端口，默认 8089（睿尔曼默认上报端口）
        arm_dof : int
            机械臂自由度，默认 7（RM65/RM75 为 7 轴）
        """
        self.port = port
        self.arm_dof = arm_dof
        
        # 最新状态数据（线程安全访问）
        self._joint_deg: Optional[np.ndarray] = None  # 关节角度（度）
        self._gripper_pos: Optional[int] = None       # 夹爪位置 (0-1000)
        self._eef_pose: Optional[np.ndarray] = None   # 末端位姿 [x, y, z, rx, ry, rz]（米/弧度）
        self._last_recv_ts: Optional[float] = None    # 最后接收时间戳
        self._lock = threading.Lock()
        
        # 后台接收线程
        self._running = False
        self._thread: Optional[threading.Thread] = None
        self._socket: Optional[socket.socket] = None
        
        # 统计信息
        self._recv_count = 0
        self._error_count = 0
    
    def start(self) -> None:
        """
        启动 UDP 接收线程。
        
        调用后会在后台持续监听 UDP 端口，接收机械臂上报的状态数据。
        """
        if self._running:
            return
        
        # 创建 UDP socket 并绑定端口
        self._socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self._socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self._socket.bind(("0.0.0.0", self.port))
        self._socket.settimeout(0.1)  # 100ms 超时，便于检查停止标志
        
        self._running = True
        self._thread = threading.Thread(target=self._recv_loop, daemon=True)
        self._thread.start()
        print(f"[UDPStateReceiver] 已启动，监听端口 {self.port}")
    
    def _recv_loop(self) -> None:
        """
        后台接收循环（内部方法）。
        
        持续接收 UDP 数据包，解析 JSON 并更新内部状态。
        """
        while self._running:
            try:
                # 接收数据（最大 64KB，足够容纳状态 JSON）
                data, addr = self._socket.recvfrom(65536)
                self._parse_and_update(data)
            except socket.timeout:
                # 超时是正常的，用于检查 _running 标志
                continue
            except Exception as exc:
                self._error_count += 1
                if self._error_count <= 5:
                    print(f"[UDPStateReceiver] 接收错误: {exc}")
    
    def _parse_and_update(self, data: bytes) -> None:
        """
        解析 UDP 数据包并更新状态（内部方法）。
        
        数据格式参考睿尔曼文档：
        - joint_status.joint_position: 关节角度，精度 0.001°
        - rm_plus_state.pos: 夹爪位置数组（RM+ 协议）
        - waypoint.position: 末端位置，数组格式 [x, y, z]，单位微米
        - waypoint.euler: 末端姿态，数组格式 [rx, ry, rz]，单位 0.001 弧度
        
        参数：
        -----
        data : bytes
            接收到的原始 UDP 数据
        """
        try:
            msg = json.loads(data.decode("utf-8"))
        except (json.JSONDecodeError, UnicodeDecodeError):
            return
        
        # 兼容：如果返回的是 list，跳过（某些固件可能返回数组）
        if not isinstance(msg, dict):
            return
        
        # 验证是否为机械臂状态上报消息
        if msg.get("state") != "realtime_arm_joint_state":
            return
        
        now = time.time()
        joint_deg = None
        gripper_pos = None
        
        # 解析关节角度
        # joint_status.joint_position 单位为 0.001°，需要除以 1000
        joint_status = msg.get("joint_status", {})
        joint_position = joint_status.get("joint_position", [])
        if joint_position and len(joint_position) >= self.arm_dof:
            # 从整数（0.001° 精度）转换为浮点度数
            joint_deg = np.array(joint_position[: self.arm_dof], dtype=float) / 1000.0
        
        # 解析末端位姿 waypoint
        # 睿尔曼 UDP 上报格式：
        #   position: [x, y, z] 单位微米（需除以 1000000 转为米）
        #   euler: [rx, ry, rz] 单位 0.001 弧度（需除以 1000 转为弧度）
        eef_pose = None
        waypoint = msg.get("waypoint", {})
        if waypoint and isinstance(waypoint, dict):
            pos = waypoint.get("position")
            euler = waypoint.get("euler")
            # 兼容数组格式和字典格式
            if isinstance(pos, list) and len(pos) >= 3 and isinstance(euler, list) and len(euler) >= 3:
                # 数组格式：position 单位微米，euler 单位 0.001 弧度
                eef_pose = np.array([
                    pos[0] / 1000000.0,   # x: 微米 -> 米
                    pos[1] / 1000000.0,   # y: 微米 -> 米
                    pos[2] / 1000000.0,   # z: 微米 -> 米
                    euler[0] / 1000.0,    # rx: 0.001rad -> rad
                    euler[1] / 1000.0,    # ry: 0.001rad -> rad
                    euler[2] / 1000.0,    # rz: 0.001rad -> rad
                ], dtype=float)
            elif isinstance(pos, dict) and isinstance(euler, dict):
                # 字典格式（兼容旧版本）：position 单位米，euler 单位弧度
                eef_pose = np.array([
                    pos.get("x", 0.0),
                    pos.get("y", 0.0),
                    pos.get("z", 0.0),
                    euler.get("rx", 0.0),
                    euler.get("ry", 0.0),
                    euler.get("rz", 0.0),
                ], dtype=float)
        
        # 解析夹爪位置
        # 方式1: 从 rm_plus_state 获取（RM+ 协议夹爪）
        rm_plus_state = msg.get("rm_plus_state")
        if rm_plus_state and isinstance(rm_plus_state, dict):
            pos_list = rm_plus_state.get("pos", [])
            if pos_list and len(pos_list) > 0:
                # 取第一个自由度的位置作为夹爪开合度
                gripper_pos = int(pos_list[0])
        
        # 方式2: 如果没有 rm_plus_state，尝试从其他字段获取
        # （某些固件版本可能使用不同字段名）
        if gripper_pos is None:
            hand = msg.get("hand")
            if hand and isinstance(hand, dict):
                hand_pos = hand.get("hand_pos", [])
                if hand_pos and len(hand_pos) > 0:
                    gripper_pos = int(hand_pos[0])
        
        # 线程安全地更新状态
        with self._lock:
            if joint_deg is not None:
                self._joint_deg = joint_deg
            if gripper_pos is not None:
                self._gripper_pos = gripper_pos
            if eef_pose is not None:
                self._eef_pose = eef_pose
            self._last_recv_ts = now
            self._recv_count += 1
    
    def read_joint_deg(self) -> Optional[np.ndarray]:
        """
        获取最新的关节角度。
        
        返回：
        -----
        Optional[np.ndarray]
            关节角度数组（单位：度），如果尚未收到数据则返回 None
        """
        with self._lock:
            return self._joint_deg.copy() if self._joint_deg is not None else None
    
    def read_gripper_pos(self) -> Optional[int]:
        """
        获取最新的夹爪位置。
        
        返回：
        -----
        Optional[int]
            夹爪位置 (0-1000)，如果尚未收到数据则返回 None
        """
        with self._lock:
            return self._gripper_pos
    
    def read_state(self) -> Tuple[Optional[np.ndarray], Optional[int]]:
        """
        一次性获取关节角度和夹爪位置。
        
        返回：
        -----
        Tuple[Optional[np.ndarray], Optional[int]]
            (关节角度, 夹爪位置) 元组
        """
        with self._lock:
            joint = self._joint_deg.copy() if self._joint_deg is not None else None
            gripper = self._gripper_pos
            return joint, gripper
    
    def read_eef_pose(self) -> Optional[np.ndarray]:
        """
        获取最新的末端位姿。
        
        返回：
        -----
        Optional[np.ndarray]
            末端位姿 [x, y, z, rx, ry, rz]（米/弧度），如果尚未收到数据则返回 None
        """
        with self._lock:
            return self._eef_pose.copy() if self._eef_pose is not None else None
    
    def read_full_state(self) -> Tuple[Optional[np.ndarray], Optional[int], Optional[np.ndarray]]:
        """
        一次性获取关节角度、夹爪位置和末端位姿。
        
        返回：
        -----
        Tuple[Optional[np.ndarray], Optional[int], Optional[np.ndarray]]
            (关节角度, 夹爪位置, 末端位姿) 元组
        """
        with self._lock:
            joint = self._joint_deg.copy() if self._joint_deg is not None else None
            gripper = self._gripper_pos
            eef = self._eef_pose.copy() if self._eef_pose is not None else None
            return joint, gripper, eef
    
    def get_stats(self) -> dict:
        """
        获取接收统计信息。
        
        返回：
        -----
        dict
            包含 recv_count（接收计数）、error_count（错误计数）、
            last_recv_ts（最后接收时间）的字典
        """
        with self._lock:
            return {
                "recv_count": self._recv_count,
                "error_count": self._error_count,
                "last_recv_ts": self._last_recv_ts,
            }
    
    def is_receiving(self, timeout: float = 1.0) -> bool:
        """
        检查是否正在正常接收数据。
        
        参数：
        -----
        timeout : float
            判定超时时间（秒），如果超过该时间未收到数据则认为异常
        
        返回：
        -----
        bool
            True 表示正常接收，False 表示可能断连
        """
        with self._lock:
            if self._last_recv_ts is None:
                return False
            return (time.time() - self._last_recv_ts) < timeout
    
    def stop(self) -> None:
        """
        停止 UDP 接收线程。
        
        程序结束前应调用此方法释放资源。
        """
        self._running = False
        if self._thread is not None:
            self._thread.join(timeout=1.0)
            self._thread = None
        if self._socket is not None:
            try:
                self._socket.close()
            except Exception:
                pass
            self._socket = None
        print(f"[UDPStateReceiver] 已停止，共接收 {self._recv_count} 条消息")
