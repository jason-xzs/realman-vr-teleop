"""
VR 二进制数据解析模块 - 解析 UDP 12345 接收的二进制数据

移植自 hmi-user-dev/web/backend/ROS_Middleware/vr_udp_arm_middle_node.py

二进制数据格式:
    Magic: 0x56524441 ("VRDA")
    总大小: ~1624 bytes
    
    结构:
    - Header (13 bytes): magic(4) + version(1) + timestamp(4) + frame(4)
    - HMD (41 bytes): position(12) + rotation(16) + mounted(1) + ipd(4) + eye_height(4)
    - Left Controller (46 bytes): position(12) + rotation(16) + grip(4) + trigger(4) + buttons(2) + joystick(8)
    - Right Controller (46 bytes): 同上
    - Hand Tracking (2×739 bytes): 可选
"""
import struct
import logging
from typing import Optional, Dict, Any, Tuple

logger = logging.getLogger(__name__)

# 二进制格式常量
MAGIC_NUMBER = 0x56524441  # "VRDA"
EXPECTED_SIZE = 1624


def read_vector3(data: bytes, offset: int) -> Tuple[Dict[str, float], int]:
    """读取 Vector3 (12 bytes: 3 floats)"""
    x, y, z = struct.unpack_from('fff', data, offset)
    return {'x': x, 'y': y, 'z': z}, offset + 12


def read_quaternion(data: bytes, offset: int) -> Tuple[Dict[str, float], int]:
    """读取 Quaternion (16 bytes: 4 floats)"""
    x, y, z, w = struct.unpack_from('ffff', data, offset)
    return {'x': x, 'y': y, 'z': z, 'w': w}, offset + 16


def parse_hand_data_flexible(data: bytes, offset: int, max_size: int) -> Tuple[Dict, int]:
    """解析手部数据（可变大小）"""
    start_offset = offset
    
    # 基本信息
    tracked, = struct.unpack_from('B', data, offset)
    offset += 1
    confidence, = struct.unpack_from('B', data, offset)
    offset += 1
    scale, = struct.unpack_from('f', data, offset)
    offset += 4
    
    # 手指捏合状态
    pinch_thumb, = struct.unpack_from('B', data, offset)
    offset += 1
    pinch_index, = struct.unpack_from('B', data, offset)
    offset += 1
    pinch_middle, = struct.unpack_from('B', data, offset)
    offset += 1
    pinch_ring, = struct.unpack_from('B', data, offset)
    offset += 1
    pinch_pinky, = struct.unpack_from('B', data, offset)
    offset += 1
    
    # 计算可用数据大小和关节数量
    used_size = offset - start_offset
    available_size = max_size - used_size
    joint_size = 28  # 每个关节 28 字节
    max_joints = available_size // joint_size
    
    # 关节数据
    joints = []
    for i in range(max_joints):
        if offset + joint_size > len(data):
            break
        try:
            position, offset = read_vector3(data, offset)
            rotation, offset = read_quaternion(data, offset)
            joints.append({'position': position, 'rotation': rotation})
        except:
            break
    
    return {
        'tracked': bool(tracked),
        'confidence': bool(confidence),
        'scale': scale,
        'pinch': {
            'thumb': bool(pinch_thumb),
            'index': bool(pinch_index),
            'middle': bool(pinch_middle),
            'ring': bool(pinch_ring),
            'pinky': bool(pinch_pinky)
        },
        'joints': joints
    }, offset


def parse_binary_vr_data(data: bytes) -> Optional[Dict[str, Any]]:
    """
    解析二进制 VR 数据包
    
    Args:
        data: 原始二进制数据
        
    Returns:
        解析后的数据字典，失败返回 None
    """
    try:
        actual_size = len(data)
        
        # 允许 ±10 字节的差异
        if actual_size < EXPECTED_SIZE - 10:
            logger.warning(f"数据包大小不足: {actual_size} < {EXPECTED_SIZE - 10}")
            return None
        
        offset = 0
        
        # 解析头部
        magic, = struct.unpack_from('I', data, offset)
        offset += 4
        if magic != MAGIC_NUMBER:
            logger.warning(f"无效的 magic number: 0x{magic:08X}")
            return None
        
        version, = struct.unpack_from('B', data, offset)
        offset += 1
        
        timestamp, = struct.unpack_from('f', data, offset)
        offset += 4
        frame, = struct.unpack_from('i', data, offset)
        offset += 4
        
        result = {
            'timestamp': timestamp,
            'frame': frame
        }
        
        # HMD 数据 (41 bytes)
        hmd_pos, offset = read_vector3(data, offset)
        hmd_rot, offset = read_quaternion(data, offset)
        mounted, = struct.unpack_from('B', data, offset)
        offset += 1
        ipd, = struct.unpack_from('f', data, offset)
        offset += 4
        eye_height, = struct.unpack_from('f', data, offset)
        offset += 4
        
        result['hmd'] = {
            'position': hmd_pos,
            'rotation': hmd_rot,
            'mounted': bool(mounted),
            'ipd': ipd,
            'eyeHeight': eye_height
        }
        
        # 左手控制器数据 (46 bytes)
        left_pos, offset = read_vector3(data, offset)
        left_rot, offset = read_quaternion(data, offset)
        left_grip, = struct.unpack_from('f', data, offset)
        offset += 4
        left_trigger, = struct.unpack_from('f', data, offset)
        offset += 4
        left_btnA, = struct.unpack_from('B', data, offset)
        offset += 1
        left_btnB, = struct.unpack_from('B', data, offset)
        offset += 1
        left_stick_x, = struct.unpack_from('f', data, offset)
        offset += 4
        left_stick_y, = struct.unpack_from('f', data, offset)
        offset += 4
        
        # 右手控制器数据 (46 bytes)
        right_pos, offset = read_vector3(data, offset)
        right_rot, offset = read_quaternion(data, offset)
        right_grip, = struct.unpack_from('f', data, offset)
        offset += 4
        right_trigger, = struct.unpack_from('f', data, offset)
        offset += 4
        right_btnA, = struct.unpack_from('B', data, offset)
        offset += 1
        right_btnB, = struct.unpack_from('B', data, offset)
        offset += 1
        right_stick_x, = struct.unpack_from('f', data, offset)
        offset += 4
        right_stick_y, = struct.unpack_from('f', data, offset)
        offset += 4
        
        result['controllers'] = {
            'left': {
                'position': left_pos,
                'rotation': left_rot,
                'grip': left_grip,
                'trigger': left_trigger,
                'buttonA': bool(left_btnA),
                'buttonB': bool(left_btnB),
                'joystick': {'x': left_stick_x, 'y': left_stick_y}
            },
            'right': {
                'position': right_pos,
                'rotation': right_rot,
                'grip': right_grip,
                'trigger': right_trigger,
                'buttonA': bool(right_btnA),
                'buttonB': bool(right_btnB),
                'joystick': {'x': right_stick_x, 'y': right_stick_y}
            }
        }
        
        # 手部追踪数据（可选）
        remaining_data = len(data) - offset
        hand_data_size = remaining_data // 2
        
        # 左手数据
        try:
            if remaining_data >= 11:
                left_hand, offset = parse_hand_data_flexible(data, offset, hand_data_size)
                result['leftHand'] = left_hand
            else:
                result['leftHand'] = {'tracked': False}
        except Exception:
            result['leftHand'] = {'tracked': False}
        
        # 右手数据
        try:
            remaining_data = len(data) - offset
            if remaining_data >= 11:
                right_hand, offset = parse_hand_data_flexible(data, offset, remaining_data)
                result['rightHand'] = right_hand
            else:
                result['rightHand'] = {'tracked': False}
        except Exception:
            result['rightHand'] = {'tracked': False}
        
        return result
        
    except Exception as e:
        logger.error(f"解析二进制数据失败: {e}")
        return None


def convert_binary_to_vr_format(binary_data: Optional[Dict]) -> Optional[Dict[str, Any]]:
    """
    将二进制解析结果转换为统一 VR 数据格式
    
    Args:
        binary_data: parse_binary_vr_data 的返回值
        
    Returns:
        统一格式的 VR 数据
    """
    if not binary_data:
        return None
    
    vr_data = {}
    
    # HMD 数据转换
    if 'hmd' in binary_data:
        hmd = binary_data['hmd']
        vr_data['headset'] = {
            'position': (hmd['position']['x'], hmd['position']['y'], hmd['position']['z']),
            'rotation': (hmd['rotation']['x'], hmd['rotation']['y'], 
                        hmd['rotation']['z'], hmd['rotation']['w']),
            'mounted': 1 if hmd['mounted'] else 0,
            'ipd': hmd['ipd'],
            'eye_height': hmd['eyeHeight']
        }
    
    # 控制器数据转换
    if 'controllers' in binary_data:
        ctrl = binary_data['controllers']
        
        # 左手控制器: buttonA -> X, buttonB -> Y
        vr_data['left_controller'] = {
            'position': (ctrl['left']['position']['x'], ctrl['left']['position']['y'], 
                        ctrl['left']['position']['z']),
            'rotation': (ctrl['left']['rotation']['x'], ctrl['left']['rotation']['y'],
                        ctrl['left']['rotation']['z'], ctrl['left']['rotation']['w']),
            'joystick': (ctrl['left']['joystick']['x'], ctrl['left']['joystick']['y']),
            'buttons': {
                'X': 1 if ctrl['left']['buttonA'] else 0,
                'Y': 1 if ctrl['left']['buttonB'] else 0,
                'JoyStick': 0
            },
            'grip': ctrl['left']['grip'],
            'trigger': ctrl['left']['trigger']
        }
        
        # 右手控制器: buttonA -> A, buttonB -> B
        vr_data['right_controller'] = {
            'position': (ctrl['right']['position']['x'], ctrl['right']['position']['y'],
                        ctrl['right']['position']['z']),
            'rotation': (ctrl['right']['rotation']['x'], ctrl['right']['rotation']['y'],
                        ctrl['right']['rotation']['z'], ctrl['right']['rotation']['w']),
            'joystick': (ctrl['right']['joystick']['x'], ctrl['right']['joystick']['y']),
            'buttons': {
                'A': 1 if ctrl['right']['buttonA'] else 0,
                'B': 1 if ctrl['right']['buttonB'] else 0,
                'JoyStick': 0
            },
            'grip': ctrl['right']['grip'],
            'trigger': ctrl['right']['trigger']
        }
    
    return vr_data


# 测试代码
if __name__ == "__main__":
    import socket
    
    logging.basicConfig(level=logging.INFO)
    
    # 创建 UDP socket 监听
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.bind(("0.0.0.0", 12345))
    sock.settimeout(1.0)
    
    print("等待 UDP 12345 数据...")
    
    try:
        while True:
            try:
                data, addr = sock.recvfrom(65535)
                binary_data = parse_binary_vr_data(data)
                if binary_data:
                    vr_data = convert_binary_to_vr_format(binary_data)
                    if vr_data and 'right_controller' in vr_data:
                        rc = vr_data['right_controller']
                        print(f"右手柄: pos={rc['position']}, trigger={rc['trigger']:.2f}")
            except socket.timeout:
                continue
    except KeyboardInterrupt:
        print("程序已退出")
    finally:
        sock.close()
