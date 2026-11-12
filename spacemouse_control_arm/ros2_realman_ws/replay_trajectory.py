#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
LeRobot 数据集轨迹回放脚本

功能说明：
---------
读取 LeRobot 格式的数据集，将其中的 joints 轨迹在真实机械臂上回放，
用于验证采集数据的有效性和准确性。

使用方法：
---------
1. 确保机械臂已连接且处于可控状态
2. 将物体放置到与采集时相同的起始位置
3. 运行脚本：
   conda activate robocoin
   python replay_trajectory.py --dataset data/local/realman_teleop_1229_04

参数说明：
---------
--dataset: 数据集路径（相对于 lerobot_policy_deploy 目录或绝对路径）
--episode: 要回放的 episode 索引，默认 0
--speed: 回放速度倍率，默认 1.0（原速）
--dry-run: 仿真模式，不连接真实机械臂
--skip-confirm: 跳过启动确认
--ip: 机械臂 IP 地址，默认 192.168.1.18
--port: 机械臂端口，默认 8080
--num-workers: 数据加载并行进程数，默认 16

开发历程与踩坑记录：
-------------------
【版本 1 - 初始版本】问题：启动时卡死
  - 原因：get_episode_frames() 遍历整个数据集（35310帧）逐帧检查 episode_index
  - 解决：改用 ds.episode_data_index 元数据直接获取 episode 的起止索引

【版本 2 - 索引优化】问题：启动时仍然卡住
  - 原因：虽然获取了索引范围，但仍用列表推导式一次性加载所有帧 [ds[i] for i in range(from_idx, to_idx)]
  - 解决：改为惰性加载，使用 FramePrefetcher 后台线程预加载

【版本 3 - 异步预加载】问题：回放速度不受 speed_factor 控制，始终约 80 秒
  - 原因：LeRobotDataset.__getitem__ 每帧加载约 190ms，FramePrefetcher 的 buffer(60帧) 
         跟不上消费速度，get_frame() 阻塞等待数据，time.sleep() 形同虚设
  - 解决：回放前先将所有帧预加载到内存，回放时直接从内存读取

【版本 4 - 内存预加载】问题：预加载慢（约 5 帧/秒）
  - 原因：单进程逐帧加载，I/O 成为瓶颈
  - 解决：使用 torch.utils.data.DataLoader + num_workers 多进程并行加载

【版本 5 - 夹爪卡顿】问题：夹爪控制卡顿
  - 原因：使用 SDK 的 rm_set_gripper_position()，每次调用延迟约 1000ms
  - 解决：改用独立 JSON socket + RM+ 生态协议，延迟降至约 5ms
  - 注意：启动时开启 RM+ 生态，结束时关闭

关于原始数据加载机制：
-------------------
LeRobotDataset 继承自 torch.utils.data.Dataset，但其 __getitem__ 内部：
1. 从 HuggingFace Arrow 文件读取数据
2. 进行数据转换（如图像解码、归一化等）
3. 单次调用约 150-200ms（取决于数据复杂度）

使用 DataLoader(num_workers>0) 可以：
- 多进程并行执行 __getitem__
- prefetch_factor 控制预取数量
- 显著提升加载吞吐量（4-8倍）

日期：2025-12-29
更新：2026-01-04 (性能优化)
"""

import argparse
import json
import socket
import sys
import time
import threading
from queue import Queue, Empty
from pathlib import Path
from typing import Optional

import numpy as np
import torch
from torch.utils.data import DataLoader, Subset


def add_repo_paths():
    """添加项目依赖路径到 sys.path。"""
    script_dir = Path(__file__).resolve().parent
    # 项目根目录
    repo_root = script_dir.parent.parent
    
    # 添加 lerobot 路径
    lerobot_path = repo_root / "lerobot"
    if lerobot_path.exists() and str(lerobot_path) not in sys.path:
        sys.path.insert(0, str(lerobot_path))
    
    # 添加 Robotic_Arm 路径
    robotic_arm_path = repo_root / "Robotic_Arm"
    if robotic_arm_path.exists() and str(robotic_arm_path) not in sys.path:
        sys.path.insert(0, str(robotic_arm_path))
    
    # 添加项目根目录
    if str(repo_root) not in sys.path:
        sys.path.insert(0, str(repo_root))
    
    return repo_root


# 添加路径
REPO_ROOT = add_repo_paths()

# 导入 lerobot 数据集
try:
    from lerobot.datasets.lerobot_dataset import LeRobotDataset
except ImportError as exc:
    raise RuntimeError(
        "未找到 lerobot，请确认已安装或已加入 PYTHONPATH。\n"
        "可能需要运行: conda activate robocoin"
    ) from exc

# 导入机械臂控制
try:
    from Robotic_Arm.rm_ctypes_wrap import rm_thread_mode_e
    from Robotic_Arm.rm_robot_interface import RoboticArm
except ImportError as exc:
    raise RuntimeError(
        "未找到 Robotic_Arm 官方 Python API，请确认目录与依赖。"
    ) from exc


class RealmanReplayDriver:
    """
    睿尔曼机械臂回放驱动。
    
    设计原则：
    ----------
    - 关节控制：同步执行（CANFD 透传，真正非阻塞，~1ms）
    - 夹爪控制：独立 JSON socket + 独立线程（~5ms，不阻塞关节控制）
    """
    
    def __init__(
        self,
        ip: str = "192.168.1.18",
        port: int = 8080,
        follow: bool = False,
        canfd_traj_mode: int = 0,
        canfd_radio: int = 30,
        rm_plus_baud: int = 115200,
        gripper_deadband: int = 20,
        dry_run: bool = False,
    ):
        """
        初始化机械臂驱动。
        
        参数：
        -----
        ip : str
            机械臂 IP 地址
        port : int
            机械臂 TCP 端口
        follow : bool
            CANFD 跟随模式
        canfd_traj_mode : int
            CANFD 轨迹模式：0=完全透传，1=曲线拟合，2=滤波模式
        canfd_radio : int
            平滑系数
        rm_plus_baud : int
            RM+ 串口波特率（用于夹爪控制）
        gripper_deadband : int
            夹爪死区阈值，位置变化小于此值时不发送指令
        dry_run : bool
            仿真模式
        """
        self.ip = ip
        self.port = port
        self.follow = follow
        self.canfd_traj_mode = canfd_traj_mode
        self.canfd_radio = canfd_radio
        self.rm_plus_baud = rm_plus_baud
        self.dry_run = dry_run
        self.arm: Optional[RoboticArm] = None
        self.arm_dof = 7
        
        # ===== 夹爪异步控制（独立 JSON socket） =====
        self.gripper_deadband = gripper_deadband
        self._last_gripper_cmd: Optional[int] = None
        self._gripper_queue: Queue = Queue(maxsize=1)
        self._gripper_stop_event = threading.Event()
        self._gripper_thread: Optional[threading.Thread] = None
        self._json_sock: Optional[socket.socket] = None
        self._rm_plus_enabled = False

        if not dry_run:
            print(f"[RealmanReplayDriver] 正在连接机械臂 {ip}:{port}...")
            self.arm = RoboticArm(rm_thread_mode_e.RM_TRIPLE_MODE_E)
            handle = self.arm.rm_create_robot_arm(ip=ip, port=port)
            if handle is None:
                raise RuntimeError(f"无法连接机械臂 {ip}:{port}")
            
            # 设置为实时模式
            self.arm.rm_set_arm_run_mode(1)
            
            # 读取机械臂自由度
            self.arm_dof = self._read_dof()
            print(f"[RealmanReplayDriver] 连接成功，机械臂自由度: {self.arm_dof}")
            
            # ===== 创建独立 JSON socket 用于夹爪控制 =====
            if rm_plus_baud > 0:
                self._create_json_socket()
                self._enable_rm_plus()
            
            # ===== 启动夹爪异步控制线程 =====
            self._gripper_thread = threading.Thread(target=self._gripper_control_loop, daemon=True)
            self._gripper_thread.start()
            print("[RealmanReplayDriver] 夹爪异步控制线程已启动")
        else:
            print("[RealmanReplayDriver] 仿真模式，不连接真实机械臂")

    def _create_json_socket(self) -> bool:
        """创建独立的 JSON socket 用于夹爪控制。"""
        try:
            self._json_sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self._json_sock.settimeout(2.0)
            self._json_sock.connect((self.ip, self.port))
            self._json_sock.settimeout(0.5)
            print(f"[RealmanReplayDriver] JSON socket 已连接 {self.ip}:{self.port}")
            return True
        except Exception as exc:
            print(f"[RealmanReplayDriver] JSON socket 连接失败: {exc}")
            self._json_sock = None
            return False

    def _send_json_cmd(self, cmd: dict) -> Optional[dict]:
        """发送 JSON 命令并接收响应。"""
        if self._json_sock is None:
            return None
        try:
            data = json.dumps(cmd).encode("utf-8")
            self._json_sock.sendall(data)
            resp = self._json_sock.recv(4096)
            return json.loads(resp.decode("utf-8"))
        except Exception:
            return None

    def _enable_rm_plus(self) -> bool:
        """开启 RM+ 末端生态协议。"""
        if self.arm is None:
            return False
        ret = self.arm.rm_set_rm_plus_mode(self.rm_plus_baud)
        if ret == 0:
            self._rm_plus_enabled = True
            print(f"[RealmanReplayDriver] RM+ 生态已开启 (波特率={self.rm_plus_baud})")
            return True
        print(f"[RealmanReplayDriver] RM+ 开启失败: ret={ret}")
        return False

    def _disable_rm_plus(self) -> bool:
        """关闭 RM+ 末端生态协议。"""
        if self.arm is None or not self._rm_plus_enabled:
            return True
        ret = self.arm.rm_set_rm_plus_mode(0)
        if ret == 0:
            self._rm_plus_enabled = False
            print("[RealmanReplayDriver] RM+ 生态已关闭")
            return True
        print(f"[RealmanReplayDriver] RM+ 关闭失败: ret={ret}")
        return False

    def _send_gripper_json(self, position: int) -> int:
        """通过 JSON 协议发送夹爪指令（~5ms）。"""
        if self._json_sock is None:
            return -1
        cmd = {"command": "set_gripper_position", "position": int(position), "block": False}
        resp = self._send_json_cmd(cmd)
        return 0 if resp and resp.get("arm_err") == 0 else -1

    def _read_dof(self) -> int:
        """读取机械臂自由度。"""
        if self.arm is None:
            return 7
        code, info = self.arm.rm_get_robot_info()
        dof = info.arm_dof if hasattr(info, "arm_dof") else 0
        if dof == 0:
            _, joint_deg = self.arm.rm_get_joint_degree()
            dof = len(joint_deg) if joint_deg else 7
        return dof

    def get_current_joint_deg(self) -> Optional[np.ndarray]:
        """获取当前关节角度（度）。"""
        if self.dry_run or self.arm is None:
            return None
        try:
            code, joint_deg = self.arm.rm_get_joint_degree()
            if code == 0 and joint_deg:
                return np.array(joint_deg[:self.arm_dof], dtype=np.float64)
        except Exception as exc:
            print(f"[RealmanReplayDriver] 读取关节角度失败: {exc}")
        return None

    def send_joint_deg(self, joint_deg: np.ndarray) -> int:
        """
        发送关节角度指令（非阻塞）。
        
        参数：
        -----
        joint_deg : np.ndarray
            目标关节角度（单位：度）
        
        返回：
        -----
        int
            0=成功，其他=失败
        """
        if self.dry_run:
            return 0
        if self.arm is None:
            return -1
        target = list(joint_deg[: self.arm_dof])
        return self.arm.rm_movej_canfd(
            target, self.follow, 0.0, self.canfd_traj_mode, self.canfd_radio
        )

    def _gripper_control_loop(self):
        """
        夹爪控制线程：独立于主循环，不阻塞关节控制。
        
        使用 JSON 协议发送夹爪指令（~5ms），比 SDK 方法快 100+ 倍。
        """
        while not self._gripper_stop_event.is_set():
            try:
                gripper_pos = self._gripper_queue.get(timeout=0.1)
                self._send_gripper_json(gripper_pos)
            except Empty:
                continue
            except Exception as exc:
                print(f"[RealmanReplayDriver] 夹爪控制异常: {exc}")

    def send_gripper_async(self, position: int) -> None:
        """
        异步发送夹爪位置指令（放入队列，由独立线程处理）。
        
        此方法立即返回，不阻塞调用方。夹爪指令会被放入队列，
        由独立的夹爪控制线程处理。自动应用死区判断。
        
        参数：
        -----
        position : int
            目标夹爪位置 (0-1000)
        """
        if self.dry_run:
            return
        
        # 死区判断：只有当位置变化超过阈值时才放入队列
        if self._last_gripper_cmd is not None:
            if abs(position - self._last_gripper_cmd) < self.gripper_deadband:
                return
        
        # 非阻塞放入队列，如果队列满则丢弃旧指令（只保留最新）
        try:
            # 先清空队列中的旧指令
            while not self._gripper_queue.empty():
                try:
                    self._gripper_queue.get_nowait()
                except Empty:
                    break
            self._gripper_queue.put_nowait(position)
            self._last_gripper_cmd = position
        except Exception:
            pass  # 队列操作失败时静默忽略

    def send_gripper_sync(self, position: int) -> int:
        """
        同步发送夹爪位置指令（直接调用，会阻塞）。
        
        用于初始化等需要等待夹爪到位的场景。
        
        参数：
        -----
        position : int
            目标夹爪位置 (0-1000)
        
        返回：
        -----
        int
            0=成功，其他=失败
        """
        if self.dry_run:
            return 0
        self._last_gripper_cmd = position
        return self._send_gripper_json(position)

    def move_to_start(self, joint_deg: np.ndarray, duration: float = 3.0, hz: float = 50.0) -> bool:
        """
        移动到起始位置（使用 CANFD 透传，渐进式插值）。
        
        通过线性插值从当前位置平滑移动到目标位置，
        使用 CANFD 透传而非阻塞式 movej，更可靠。
        
        参数：
        -----
        joint_deg : np.ndarray
            目标关节角度（度）
        duration : float
            移动时间（秒），默认 3.0 秒
        hz : float
            插值频率（Hz），默认 50Hz
        
        返回：
        -----
        bool
            True=成功，False=失败
        """
        if self.dry_run:
            print(f"[RealmanReplayDriver] 仿真模式：移动到起始位置 {joint_deg}")
            return True
        if self.arm is None:
            return False
        
        # 获取当前关节角度
        current = self.get_current_joint_deg()
        if current is None:
            print("[RealmanReplayDriver] 无法获取当前关节角度")
            return False
        
        target = np.array([float(x) for x in joint_deg[: self.arm_dof]])
        
        # 计算插值步数
        num_steps = int(duration * hz)
        interval = 1.0 / hz
        
        print(f"[RealmanReplayDriver] 使用 CANFD 透传移动到起始位置 ({num_steps} 步, {duration}秒)")
        
        # 线性插值移动
        for i in range(num_steps + 1):
            t = i / num_steps  # 0.0 ~ 1.0
            # 使用平滑插值（ease in-out）
            t_smooth = t * t * (3.0 - 2.0 * t)
            interp = current + (target - current) * t_smooth
            
            rc = self.arm.rm_movej_canfd(
                list(interp), self.follow, 0.0, self.canfd_traj_mode, self.canfd_radio
            )
            if rc != 0:
                print(f"\n[RealmanReplayDriver] CANFD 透传失败，步数 {i}/{num_steps}，返回码: {rc}")
                # 继续尝试，不立即失败
            
            time.sleep(interval)
        
        # 最后确保到达目标位置
        time.sleep(0.2)
        return True

    def close(self):
        """关闭机械臂连接，释放资源。"""
        # 1. 停止夹爪控制线程
        self._gripper_stop_event.set()
        if self._gripper_thread is not None and self._gripper_thread.is_alive():
            self._gripper_thread.join(timeout=1.0)
            print("[RealmanReplayDriver] 夹爪控制线程已停止")
        
        # 2. 关闭 RM+ 生态协议
        if not self.dry_run:
            self._disable_rm_plus()
        
        # 3. 关闭 JSON socket
        if self._json_sock is not None:
            try:
                self._json_sock.close()
                print("[RealmanReplayDriver] JSON socket 已关闭")
            except Exception:
                pass
            self._json_sock = None
        
        # 4. 断开机械臂连接
        if self.arm is not None and not self.dry_run:
            try:
                self.arm.rm_delete_robot_arm()
                print("[RealmanReplayDriver] 已断开机械臂连接")
            except Exception as exc:
                print(f"[RealmanReplayDriver] 断开连接失败: {exc}")


def load_dataset(dataset_path: str) -> LeRobotDataset:
    """
    加载 LeRobot 数据集。
    
    参数：
    -----
    dataset_path : str
        数据集路径（可以是相对路径或绝对路径）
    
    返回：
    -----
    LeRobotDataset
        加载的数据集
    """
    path = Path(dataset_path)
    
    # 如果是相对路径，尝试在 data 目录下查找
    if not path.is_absolute():
        # 尝试多个可能的位置
        candidates = [
            REPO_ROOT / dataset_path,
            REPO_ROOT / "data" / dataset_path,
            Path(dataset_path),
        ]
        for candidate in candidates:
            if candidate.exists():
                path = candidate
                break
    
    if not path.exists():
        raise FileNotFoundError(f"数据集不存在: {dataset_path}")
    
    # 从路径中提取 repo_id（最后两级目录）
    repo_id = f"{path.parent.name}/{path.name}"
    
    print(f"[load_dataset] 加载数据集: {path}")
    print(f"[load_dataset] repo_id: {repo_id}")
    
    ds = LeRobotDataset(repo_id, root=str(path))
    
    print(f"[load_dataset] 数据集大小: {len(ds)} 帧")
    print(f"[load_dataset] FPS: {ds.fps}")
    print(f"[load_dataset] 总时长: {len(ds) / ds.fps:.2f} 秒")
    
    return ds


def get_episode_index_range(ds: LeRobotDataset, episode_idx: int) -> tuple:
    """
    获取指定 episode 的帧索引范围（不加载数据）。
    
    参数：
    -----
    ds : LeRobotDataset
        数据集
    episode_idx : int
        episode 索引
    
    返回：
    -----
    tuple
        (from_idx, to_idx) 帧索引范围
    """
    from_idx = ds.episode_data_index["from"][episode_idx].item()
    to_idx = ds.episode_data_index["to"][episode_idx].item()
    return from_idx, to_idx


class FramePrefetcher:
    """
    帧预加载器：后台线程提前加载帧数据，避免回放卡顿。
    """
    def __init__(self, ds: LeRobotDataset, from_idx: int, to_idx: int, buffer_size: int = 60):
        """
        参数：
        -----
        ds : LeRobotDataset
            数据集
        from_idx : int
            起始帧索引
        to_idx : int
            结束帧索引
        buffer_size : int
            预加载缓冲区大小（帧数）
        """
        self.ds = ds
        self.from_idx = from_idx
        self.to_idx = to_idx
        self.buffer = Queue(maxsize=buffer_size)
        self.stop_event = threading.Event()
        self.thread = threading.Thread(target=self._prefetch_worker, daemon=True)
        self.thread.start()
    
    def _prefetch_worker(self):
        """后台预加载线程。"""
        for idx in range(self.from_idx, self.to_idx):
            if self.stop_event.is_set():
                break
            frame = self.ds[idx]
            self.buffer.put(frame)  # 阻塞直到有空间
    
    def get_frame(self, timeout: float = 1.0):
        """获取下一帧，阻塞等待。"""
        return self.buffer.get(timeout=timeout)
    
    def stop(self):
        """停止预加载。"""
        self.stop_event.set()


def rad_to_deg(rad: np.ndarray) -> np.ndarray:
    """将弧度转换为度。"""
    return np.degrees(rad)


def gripper_normalized_to_position(normalized: float) -> int:
    """
    将归一化夹爪位置转换为实际位置。
    
    参数：
    -----
    normalized : float
        归一化位置 (0.0 ~ 1.0)
    
    返回：
    -----
    int
        实际位置 (0 ~ 1000)
    """
    # 限制在 0~1 范围内
    normalized = max(0.0, min(1.0, normalized))
    return int(normalized * 1000)


def replay_trajectory(
    ds: LeRobotDataset,
    driver: RealmanReplayDriver,
    episode_idx: int = 0,
    speed_factor: float = 1.0,
    skip_confirm: bool = False,
    num_workers: int = 0,
):
    """
    回放轨迹。
    
    参数：
    -----
    ds : LeRobotDataset
        数据集
    driver : RealmanReplayDriver
        机械臂驱动
    episode_idx : int
        要回放的 episode 索引
    speed_factor : float
        速度倍率
    skip_confirm : bool
        跳过启动确认
    num_workers : int
        数据加载的并行进程数，0 表示单进程
    """
    print(f"\n{'='*60}")
    print(f"轨迹回放 - Episode {episode_idx}")
    print(f"{'='*60}")
    
    # 获取该 episode 的帧索引范围（不加载数据）
    from_idx, to_idx = get_episode_index_range(ds, episode_idx)
    num_frames = to_idx - from_idx
    if num_frames <= 0:
        raise ValueError(f"Episode {episode_idx} 不存在或没有帧数据")
    
    print(f"Episode {episode_idx} 共 {num_frames} 帧")
    
    # 计算回放参数
    fps = ds.fps
    frame_interval = 1.0 / fps / speed_factor
    total_duration = num_frames / fps / speed_factor
    
    print(f"原始 FPS: {fps}")
    print(f"速度倍率: {speed_factor}x")
    print(f"回放帧间隔: {frame_interval * 1000:.2f} ms")
    print(f"预计回放时长: {total_duration:.2f} 秒")
    
    # 获取起始位置（只加载第一帧）
    first_frame = ds[from_idx]
    first_state = first_frame["observation.state"].numpy()
    first_joint_rad = first_state[:7]
    first_joint_deg = rad_to_deg(first_joint_rad)
    first_gripper = gripper_normalized_to_position(first_state[7])
    
    print(f"\n起始关节角度（弧度）: {first_joint_rad}")
    print(f"起始关节角度（度）  : {first_joint_deg}")
    print(f"起始夹爪位置: {first_gripper}")
    
    # 获取当前位置
    current_joint = driver.get_current_joint_deg()
    if current_joint is not None:
        print(f"\n当前关节角度（度）: {current_joint}")
        diff = np.abs(first_joint_deg - current_joint)
        print(f"与起始位置差值（度）: {diff}")
        max_diff = np.max(diff)
        print(f"最大差值: {max_diff:.2f}°")
    
    # 确认开始
    if not skip_confirm:
        print(f"\n{'='*60}")
        print("警告：即将开始轨迹回放！")
        print("请确保：")
        print("  1. 机械臂周围无障碍物")
        print("  2. 急停按钮在手边")
        print("  3. 物体已放置到正确位置")
        print(f"{'='*60}")
        confirm = input("\n输入 'yes' 确认开始回放: ").strip().lower()
        if confirm != "yes":
            print("已取消回放")
            return
    
    # 移动到起始位置（使用 CANFD 透传，3秒平滑过渡）
    print("\n[1/3] 正在移动到起始位置...")
    if not driver.move_to_start(first_joint_deg, duration=3.0):
        raise RuntimeError("移动到起始位置失败")
    print("已到达起始位置")
    
    # 设置夹爪初始位置（同步等待）
    print(f"[2/3] 设置夹爪初始位置: {first_gripper}")
    driver.send_gripper_sync(first_gripper)
    time.sleep(1.0)  # 等待夹爪到位
    
    if not skip_confirm:
        input("\n按回车开始轨迹回放...")
    
    # 预加载所有帧到内存（避免回放时 I/O 阻塞）
    print(f"\n正在预加载 {num_frames} 帧数据 (num_workers={num_workers})...")
    load_start = time.time()
    
    if num_workers > 0:
        # 使用 DataLoader 多进程加载
        subset = Subset(ds, range(from_idx, to_idx))
        loader = DataLoader(subset, batch_size=1, num_workers=num_workers, prefetch_factor=2)
        frames = []
        for i, batch in enumerate(loader):
            # batch 是字典，每个值可能是 tensor 或 list，需要处理
            frame = {}
            for k, v in batch.items():
                if isinstance(v, torch.Tensor):
                    frame[k] = v.squeeze(0)
                elif isinstance(v, list) and len(v) == 1:
                    frame[k] = v[0]
                else:
                    frame[k] = v
            frames.append(frame)
            if (i + 1) % 50 == 0 or i == num_frames - 1:
                print(f"\r预加载进度: {i + 1}/{num_frames}", end="", flush=True)
    else:
        # 单进程加载
        frames = []
        for i, ds_idx in enumerate(range(from_idx, to_idx)):
            frames.append(ds[ds_idx])
            if (i + 1) % 50 == 0 or i == num_frames - 1:
                print(f"\r预加载进度: {i + 1}/{num_frames}", end="", flush=True)
    
    load_elapsed = time.time() - load_start
    print(f"\n预加载完成！耗时: {load_elapsed:.2f}s ({num_frames / load_elapsed:.1f} 帧/秒)")
    
    # 开始回放
    print(f"\n[3/3] 开始回放轨迹 ({num_frames} 帧)...")
    print("      关节控制: 同步执行 (CANFD 透传)")
    print("      夹爪控制: 异步执行 (独立线程)")
    start_time = time.time()
    
    try:
        for frame_idx, frame in enumerate(frames):
            frame_start = time.time()
            
            # 获取目标状态（使用 action 而非 state，action 是期望位置）
            action = frame["action"].numpy()
            joint_rad = action[:7]
            joint_deg = rad_to_deg(joint_rad)
            gripper_pos = gripper_normalized_to_position(action[7])
            
            # 1. 发送关节角度（同步，CANFD 透传是真正非阻塞的，~1ms）
            rc = driver.send_joint_deg(joint_deg)
            if rc != 0:
                print(f"\n警告：第 {frame_idx} 帧关节指令发送失败，返回码: {rc}")
            
            # 2. 发送夹爪位置（异步，放入队列由独立线程处理，不阻塞）
            #    死区判断已在 send_gripper_async 内部实现
            driver.send_gripper_async(gripper_pos)
            
            # 进度显示
            if frame_idx % 30 == 0 or frame_idx == num_frames - 1:
                elapsed = time.time() - start_time
                progress = (frame_idx + 1) / num_frames * 100
                print(f"\r进度: {progress:5.1f}% ({frame_idx + 1}/{num_frames}) "
                      f"已用时: {elapsed:.1f}s", end="", flush=True)
            
            # 控制帧率
            frame_elapsed = time.time() - frame_start
            sleep_time = frame_interval - frame_elapsed
            if sleep_time > 0:
                time.sleep(sleep_time)
    
    except KeyboardInterrupt:
        print("\n\n用户中断回放")
    
    total_elapsed = time.time() - start_time
    print(f"\n\n回放完成！")
    print(f"实际用时: {total_elapsed:.2f} 秒")
    print(f"预期用时: {total_duration:.2f} 秒")
    print(f"时间偏差: {(total_elapsed - total_duration) * 1000:.1f} ms")


def main():
    parser = argparse.ArgumentParser(
        description="LeRobot 数据集轨迹回放脚本",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
示例用法：
  # 回放默认数据集
  python replay_trajectory.py --dataset data/local/realman_teleop_1229_04
  
  # 仿真模式测试
  python replay_trajectory.py --dataset data/local/realman_teleop_1229_04 --dry-run
  
  # 0.5倍速回放
  python replay_trajectory.py --dataset data/local/realman_teleop_1229_04 --speed 0.5
  
  # 回放第2个episode
  python replay_trajectory.py --dataset data/local/realman_teleop_1229_04 --episode 1
        """
    )
    
    parser.add_argument(
        "--dataset", "-d",
        type=str,
        required=True,
        help="数据集路径（相对于项目根目录或绝对路径）"
    )
    parser.add_argument(
        "--episode", "-e",
        type=int,
        default=0,
        help="要回放的 episode 索引，默认 0"
    )
    parser.add_argument(
        "--speed", "-s",
        type=float,
        default=1,
        help="回放速度倍率，默认 1.0（原速），0.5 为半速，2.0 为两倍速"
    )
    parser.add_argument(
        "--dry-run",
        action="store_true",
        help="仿真模式，不连接真实机械臂"
    )
    parser.add_argument(
        "--skip-confirm",
        action="store_true",
        help="跳过启动确认"
    )
    parser.add_argument(
        "--ip",
        type=str,
        default="192.168.1.18",
        help="机械臂 IP 地址，默认 192.168.1.18"
    )
    parser.add_argument(
        "--port",
        type=int,
        default=8080,
        help="机械臂端口，默认 8080"
    )
    parser.add_argument(
        "--follow",
        action="store_true",
        help="使用高跟随模式（要求透传周期 ≤10ms）"
    )
    parser.add_argument(
        "--num-workers", "-w",
        type=int,
        default=16,
        help="数据加载的并行进程数，默认 4，设为 0 使用单进程"
    )
    
    args = parser.parse_args()
    
    # 参数验证
    if args.speed <= 0:
        raise ValueError("速度倍率必须大于 0")
    
    print("="*60)
    print("LeRobot 数据集轨迹回放工具")
    print("="*60)
    print(f"数据集: {args.dataset}")
    print(f"Episode: {args.episode}")
    print(f"速度倍率: {args.speed}x")
    print(f"机械臂: {args.ip}:{args.port}")
    print(f"模式: {'仿真' if args.dry_run else '真机'}")
    print("="*60)
    
    # 加载数据集
    ds = load_dataset(args.dataset)
    
    # 创建机械臂驱动
    driver = RealmanReplayDriver(
        ip=args.ip,
        port=args.port,
        follow=args.follow,
        dry_run=args.dry_run,
    )
    
    try:
        # 执行回放
        replay_trajectory(
            ds=ds,
            driver=driver,
            episode_idx=args.episode,
            speed_factor=args.speed,
            skip_confirm=args.skip_confirm,
            num_workers=args.num_workers,
        )
    finally:
        # 关闭连接
        driver.close()


if __name__ == "__main__":
    main()
