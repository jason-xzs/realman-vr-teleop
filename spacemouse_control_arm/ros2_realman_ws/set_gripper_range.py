#!/usr/bin/env python3
"""
夹爪行程设置脚本

功能：
1. 交互式控制夹爪开合位置
2. 设置夹爪的最大行程（完全张开位置）
3. 设置夹爪的最小行程（完全闭合位置）
4. 保存行程配置到 JSON 文件

使用方法：
    cd /home/ubuntu/Desktop/Workspace/lerobot_policy_deploy/spacemouse_control_arm/ros2_realman_ws
    python set_gripper_range.py

按键说明：
    o / O : 夹爪完全张开 (1000)
    c / C : 夹爪完全闭合 (0)
    + / = : 增加夹爪开度 (+50)
    - / _ : 减少夹爪开度 (-50)
    数字  : 直接输入目标位置 (0-1000)
    m     : 设置当前位置为最大行程
    n     : 设置当前位置为最小行程
    s     : 保存当前行程配置
    p     : 显示当前行程配置
    q     : 退出程序

注意：需要机械臂已连接！
"""

import sys
import json
import time
import socket
from pathlib import Path

# 添加机械臂 SDK 路径
sys.path.insert(0, "/home/ubuntu/Desktop/Workspace/lerobot_policy_deploy")

from Robotic_Arm.rm_robot_interface import RoboticArm, rm_thread_mode_e


# ===== 配置参数 =====
ARM_IP = "192.168.1.18"
ARM_PORT = 8080
CONFIG_FILE = Path(__file__).parent / "gripper_range_config.json"


class GripperController:
    """夹爪控制器，使用 JSON 协议（低延迟）"""
    
    def __init__(self, ip: str, port: int):
        self.ip = ip
        self.port = port
        self.arm: RoboticArm = None
        self._json_sock: socket.socket = None
        
        # 行程配置
        self.min_position = 0     # 最小行程（完全闭合）
        self.max_position = 1000  # 最大行程（完全张开）
        self.current_position = 500  # 当前位置
    
    def connect(self) -> bool:
        """连接机械臂并开启 RM+ 协议"""
        print(f"正在连接机械臂 {self.ip}:{self.port}...")
        
        try:
            # 1. 创建 SDK 连接
            self.arm = RoboticArm(rm_thread_mode_e.RM_TRIPLE_MODE_E)
            self.arm.rm_create_robot_arm(ip=self.ip, port=self.port)
            self.arm.rm_set_arm_run_mode(1)
            print("  SDK 连接成功")
            
            # 2. 创建 JSON 协议 socket
            self._json_sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self._json_sock.settimeout(2.0)
            self._json_sock.connect((self.ip, self.port))
            self._json_sock.settimeout(0.5)
            print("  JSON socket 连接成功")
            
            # 3. 开启 RM+ 生态协议（夹爪控制需要）
            cmd = {"command": "set_rm_plus_mode", "mode": 115200}
            resp = self._send_json_command(cmd)
            if resp and resp.get("set_state") is True:
                print("  RM+ 协议已开启")
            else:
                print(f"  警告：RM+ 协议开启失败，响应: {resp}")
            
            print("机械臂连接成功！\n")
            return True
            
        except Exception as e:
            print(f"连接失败: {e}")
            return False
    
    def _send_json_command(self, cmd_dict: dict, wait_response: bool = True) -> dict:
        """发送 JSON 命令"""
        if self._json_sock is None:
            return None
        
        cmd = json.dumps(cmd_dict) + "\r\n"
        try:
            self._json_sock.send(cmd.encode('utf-8'))
            
            if not wait_response:
                return {"success": True}
            
            self._json_sock.settimeout(2.0)
            data = self._json_sock.recv(4096)
            self._json_sock.settimeout(0.5)
            
            response_str = data.decode('utf-8').strip()
            if '\n' in response_str:
                response_str = response_str.split('\n')[0]
            return json.loads(response_str)
        except Exception as e:
            print(f"  命令发送失败: {e}")
            return None
    
    def set_gripper_position(self, position: int) -> bool:
        """设置夹爪位置（0-1000）"""
        # 限制在有效范围内
        position = max(0, min(1000, int(position)))
        
        cmd = {
            "command": "set_gripper_position",
            "position": position,
            "block": False
        }
        
        try:
            cmd_str = json.dumps(cmd) + "\r\n"
            self._json_sock.send(cmd_str.encode('utf-8'))
            self.current_position = position
            return True
        except Exception as e:
            print(f"夹爪命令发送失败: {e}")
            return False
    
    def save_config(self, filepath: Path = CONFIG_FILE) -> bool:
        """保存行程配置到文件"""
        config = {
            "min_position": self.min_position,
            "max_position": self.max_position,
            "description": "夹爪行程配置：0=完全闭合，1000=完全张开"
        }
        
        try:
            with open(filepath, 'w', encoding='utf-8') as f:
                json.dump(config, f, ensure_ascii=False, indent=2)
            print(f"配置已保存到: {filepath}")
            return True
        except Exception as e:
            print(f"保存配置失败: {e}")
            return False
    
    def load_config(self, filepath: Path = CONFIG_FILE) -> bool:
        """从文件加载行程配置"""
        if not filepath.exists():
            print(f"配置文件不存在: {filepath}")
            return False
        
        try:
            with open(filepath, 'r', encoding='utf-8') as f:
                config = json.load(f)
            self.min_position = config.get("min_position", 0)
            self.max_position = config.get("max_position", 1000)
            print(f"配置已加载: 最小={self.min_position}, 最大={self.max_position}")
            return True
        except Exception as e:
            print(f"加载配置失败: {e}")
            return False
    
    def close(self):
        """关闭连接"""
        # 1. 关闭 RM+ 协议
        if self._json_sock is not None:
            try:
                cmd = {"command": "set_rm_plus_mode", "mode": 0}
                self._send_json_command(cmd)
                print("RM+ 协议已关闭")
            except Exception:
                pass
            
            try:
                self._json_sock.close()
            except Exception:
                pass
        
        # 2. 关闭 SDK 连接
        if self.arm is not None:
            try:
                self.arm.rm_delete_robot_arm()
            except Exception:
                pass
        
        print("机械臂连接已断开")


def print_help():
    """打印帮助信息"""
    print("\n" + "=" * 50)
    print("🔧 夹爪行程设置工具 - 按键说明")
    print("=" * 50)
    print("  o / O     : 夹爪完全张开 (1000)")
    print("  c / C     : 夹爪完全闭合 (0)")
    print("  + / =     : 增加开度 (+50)")
    print("  - / _     : 减少开度 (-50)")
    print("  数字+回车 : 直接输入目标位置 (0-1000)")
    print("  m         : 设置当前位置为最大行程（完全张开）")
    print("  n         : 设置当前位置为最小行程（完全闭合）")
    print("  s         : 保存行程配置")
    print("  p         : 显示当前配置")
    print("  h         : 显示此帮助")
    print("  q         : 退出程序")
    print("=" * 50 + "\n")


def print_status(ctrl: GripperController):
    """打印当前状态"""
    print(f"\n📊 当前状态:")
    print(f"  当前位置: {ctrl.current_position}")
    print(f"  最小行程: {ctrl.min_position} (完全闭合)")
    print(f"  最大行程: {ctrl.max_position} (完全张开)")
    print(f"  有效范围: [{ctrl.min_position}, {ctrl.max_position}]")
    print()


def main():
    print("=" * 50)
    print("🔧 夹爪行程设置工具")
    print("=" * 50)
    
    # 创建控制器并连接
    ctrl = GripperController(ARM_IP, ARM_PORT)
    
    if not ctrl.connect():
        print("无法连接机械臂，程序退出")
        return
    
    # 尝试加载已有配置
    ctrl.load_config()
    
    # 打印帮助和状态
    print_help()
    print_status(ctrl)
    
    try:
        while True:
            try:
                user_input = input(">>> 输入命令: ").strip()
                
                if not user_input:
                    continue
                
                # 单字符命令
                cmd = user_input.lower()
                
                if cmd == 'q':
                    print("退出程序...")
                    break
                
                elif cmd == 'h':
                    print_help()
                
                elif cmd == 'p':
                    print_status(ctrl)
                
                elif cmd == 'o':
                    # 完全张开
                    pos = ctrl.max_position
                    ctrl.set_gripper_position(pos)
                    print(f"✓ 夹爪张开到: {pos}")
                
                elif cmd == 'c':
                    # 完全闭合
                    pos = ctrl.min_position
                    ctrl.set_gripper_position(pos)
                    print(f"✓ 夹爪闭合到: {pos}")
                
                elif cmd in ['+', '=']:
                    # 增加开度
                    pos = min(1000, ctrl.current_position + 50)
                    ctrl.set_gripper_position(pos)
                    print(f"✓ 夹爪开度增加到: {pos}")
                
                elif cmd in ['-', '_']:
                    # 减少开度
                    pos = max(0, ctrl.current_position - 50)
                    ctrl.set_gripper_position(pos)
                    print(f"✓ 夹爪开度减少到: {pos}")
                
                elif cmd == 'm':
                    # 设置最大行程
                    ctrl.max_position = ctrl.current_position
                    print(f"✓ 最大行程已设置为: {ctrl.max_position}")
                
                elif cmd == 'n':
                    # 设置最小行程
                    ctrl.min_position = ctrl.current_position
                    print(f"✓ 最小行程已设置为: {ctrl.min_position}")
                
                elif cmd == 's':
                    # 保存配置
                    ctrl.save_config()
                
                else:
                    # 尝试解析为数字
                    try:
                        pos = int(user_input)
                        if 0 <= pos <= 1000:
                            ctrl.set_gripper_position(pos)
                            print(f"✓ 夹爪移动到: {pos}")
                        else:
                            print("❌ 位置必须在 0-1000 范围内")
                    except ValueError:
                        print(f"❌ 未知命令: {user_input}，输入 h 查看帮助")
                
            except EOFError:
                break
                
    except KeyboardInterrupt:
        print("\n用户中断")
    finally:
        ctrl.close()
        print("程序结束")


if __name__ == "__main__":
    main()
