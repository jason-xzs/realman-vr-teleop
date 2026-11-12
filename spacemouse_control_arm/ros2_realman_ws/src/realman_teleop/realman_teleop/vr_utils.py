"""
VR 控制工具函数模块

包含：
- VRCoordinateTransform: VR → 机械臂坐标变换
- RealmanIK: 封装 Realman SDK 逆运动学接口
- PoseFilter: EMA 位置/旋转滤波器
- QuaternionUtils: 四元数工具函数
"""
from dataclasses import dataclass, field
from typing import Optional, List, Tuple
import numpy as np
from scipy.spatial.transform import Rotation as R


# ============================================================================
# 旋转映射预设方案
# ============================================================================

# 当前使用的旋转映射方案：perm_xyz_pnn
# 矩阵 M 使得 rotvec_arm = M @ rotvec_vr
# signs 用于调整旋转方向

ROTATION_MAPPING_PRESETS = {
    # ========== perm_xyz 的 8 种正负号组合 ==========
    # 基本映射: arm = (vr_x, vr_y, vr_z)，即恒等映射
    # 轴映射: VR_X→X, VR_Y→Y, VR_Z→Z

    "perm_xyz": {
        "matrix": np.array([[1, 0, 0], [0, 1, 0], [0, 0, 1]]),
        "signs": np.array([1, 1, 1]),
        "description": "恒等映射: VR_X→X, VR_Y→Y, VR_Z→Z"
    },
    "perm_xyz_npp": {
        "matrix": np.array([[1, 0, 0], [0, 1, 0], [0, 0, 1]]),
        "signs": np.array([-1, 1, 1]),
        "description": "xyz + signs(-1,1,1): VR_X→-X, VR_Y→Y, VR_Z→Z"
    },
    "perm_xyz_pnp": {
        "matrix": np.array([[1, 0, 0], [0, 1, 0], [0, 0, 1]]),
        "signs": np.array([1, -1, 1]),
        "description": "xyz + signs(1,-1,1): VR_X→X, VR_Y→-Y, VR_Z→Z"
    },
    "perm_xyz_ppn": {
        "matrix": np.array([[1, 0, 0], [0, 1, 0], [0, 0, 1]]),
        "signs": np.array([1, 1, -1]),
        "description": "xyz + signs(1,1,-1): VR_X→X, VR_Y→Y, VR_Z→-Z"
    },
    "perm_xyz_nnp": {
        "matrix": np.array([[1, 0, 0], [0, 1, 0], [0, 0, 1]]),
        "signs": np.array([-1, -1, 1]),
        "description": "xyz + signs(-1,-1,1): VR_X→-X, VR_Y→-Y, VR_Z→Z"
    },
    "perm_xyz_npn": {
        "matrix": np.array([[1, 0, 0], [0, 1, 0], [0, 0, 1]]),
        "signs": np.array([-1, 1, -1]),
        "description": "xyz + signs(-1,1,-1): VR_X→-X, VR_Y→Y, VR_Z→-Z"
    },
    "perm_xyz_pnn": {
        "matrix": np.array([[1, 0, 0], [0, 1, 0], [0, 0, 1]]),
        "signs": np.array([1, -1, -1]),
        "description": "【当前使用】xyz + signs(1,-1,-1): VR_X→X, VR_Y→-Y, VR_Z→-Z"
    },
    "perm_xyz_nnn": {
        "matrix": np.array([[1, 0, 0], [0, 1, 0], [0, 0, 1]]),
        "signs": np.array([-1, -1, -1]),
        "description": "xyz + signs(-1,-1,-1): VR_X→-X, VR_Y→-Y, VR_Z→-Z"
    },
}


# ============================================================================
# 坐标变换
# ============================================================================

class VRCoordinateTransform:
    """
    VR 控制器 → 机械臂坐标变换

    坐标系映射关系（针对左手系 VR → 右手系机械臂）：

    用户提供的坐标轴对应关系：
        VR (左手系)  →  机械臂 (右手系)
            X       →      -Y
            Y       →       Z
            Z       →       X

    右臂变换矩阵：
        [0, 0, 1]     arm_x = vr_z
        [-1, 0, 0]    arm_y = -vr_x
        [0, 1, 0]     arm_z = vr_y

    注意：这个矩阵行列式为-1（包含反射），不是纯旋转矩阵。
    位置变换直接使用矩阵乘法，旋转变换需要特殊处理手性变化。
    """

    # 右手柄 → 右臂位置变换矩阵
    # 坐标轴映射：VR_X → -ARM_Y, VR_Y → ARM_Z, VR_Z → ARM_X
    # 即：arm_x = vr_z, arm_y = -vr_x, arm_z = vr_y
    RIGHT_POSITION_MATRIX = np.array([
        [0, 0, 1],    # arm_x = vr_z
        [-1, 0, 0],   # arm_y = -vr_x
        [0, 1, 0]     # arm_z = vr_y
    ], dtype=np.float64)
    
    # 左手柄 → 左臂位置变换矩阵
    # VR 头显两只手柄的坐标约定一致：X+ 向右、Y+ 向上、Z+ 向前。
    # 左右臂使用同一套平移坐标转换，避免左右/前后在左臂被额外镜像。
    # 即：arm_x = vr_z, arm_y = -vr_x, arm_z = vr_y
    LEFT_POSITION_MATRIX = np.array([
        [0, 0, 1],    # arm_x = vr_z
        [-1, 0, 0],   # arm_y = -vr_x
        [0, 1, 0]     # arm_z = vr_y
    ], dtype=np.float64)

    # ========== 旋转变换矩阵（与位置变换分开定义）==========
    # 旋转变换需要单独定义，因为旋转轴的映射方式与位置向量不同
    #
    # 右臂旋转映射（基于用户期望的直觉控制）：
    #   绕 VR_X 旋转 → 绕 ARM_Y 旋转
    #   绕 VR_Y 旋转 → 绕 ARM_Z 旋转
    #   绕 VR_Z 旋转 → 绕 ARM_X 旋转
    RIGHT_ROTATION_MATRIX = np.array([
        [0, 0, 1],    # arm_rx = vr_rz (绕 VR_Z → 绕 ARM_X)
        [1, 0, 0],    # arm_ry = vr_rx (绕 VR_X → 绕 ARM_Y)
        [0, 1, 0]     # arm_rz = vr_ry (绕 VR_Y → 绕 ARM_Z)
    ], dtype=np.float64)

    # 左臂旋转映射：与右臂保持同一套坐标转换
    LEFT_ROTATION_MATRIX = np.array([
        [0, 0, 1],    # arm_rx = vr_rz
        [1, 0, 0],    # arm_ry = vr_rx
        [0, 1, 0]     # arm_rz = vr_ry
    ], dtype=np.float64)

    def __init__(self, side: str = "right", rotation_preset: str = "current"):
        """
        初始化坐标变换器

        参数:
            side: "left" 或 "right"
            rotation_preset: 旋转映射方案名称，参见 ROTATION_MAPPING_PRESETS
        """
        self.side = side
        self.rotation_preset = rotation_preset

        # 位置矩阵保持不变
        if side == "left":
            self.position_matrix = self.LEFT_POSITION_MATRIX
            self.rotation_matrix = self.LEFT_ROTATION_MATRIX
        else:
            self.position_matrix = self.RIGHT_POSITION_MATRIX
            self.rotation_matrix = self.RIGHT_ROTATION_MATRIX

        # 根据预设方案设置旋转参数
        if rotation_preset in ROTATION_MAPPING_PRESETS:
            preset = ROTATION_MAPPING_PRESETS[rotation_preset]
            self._rotation_base_matrix = preset["matrix"].copy()
            self._rotation_signs = preset["signs"].copy()
            self._rotation_description = preset["description"]
        else:
            # 默认使用当前实现
            self._rotation_base_matrix = self.position_matrix.copy()
            self._rotation_signs = np.array([1, 1, 1])
            self._rotation_description = "默认（位置矩阵）"

        # 为相似变换构建旋转对象
        # 位置变换矩阵 det=-1（包含反射），需要转换为有效旋转矩阵
        self._R_coord = self._matrix_to_rotation(self._rotation_base_matrix)

        print(f"[VRCoordinateTransform] 旋转映射方案: {rotation_preset}")
        print(f"  描述: {self._rotation_description}")
        print(f"  正负号: {self._rotation_signs}")

    def _matrix_to_rotation(self, matrix: np.ndarray) -> R:
        """
        将变换矩阵转换为 scipy Rotation 对象

        对于 det=-1 的矩阵（包含反射），使用 SVD 分解获取最近旋转矩阵

        参数:
            matrix: 3x3 变换矩阵

        返回:
            scipy Rotation 对象
        """
        det = np.linalg.det(matrix)
        if np.abs(det - 1.0) < 1e-6:
            # det ≈ 1，直接使用
            return R.from_matrix(matrix)
        else:
            # det ≈ -1，使用 SVD 分解获取最近旋转矩阵
            U, S, Vt = np.linalg.svd(matrix)
            R_nearest = U @ Vt
            # 确保 det = 1
            if np.linalg.det(R_nearest) < 0:
                U[:, -1] *= -1
                R_nearest = U @ Vt
            return R.from_matrix(R_nearest)
    
    def transform_position(self, position_diff: np.ndarray) -> np.ndarray:
        """
        变换位置差分向量
        
        参数:
            position_diff: VR 坐标系下的位置差分 [dx, dy, dz]
        
        返回:
            机械臂坐标系下的位置差分
        """
        return self.position_matrix @ position_diff
    
    def transform_rotation(
        self,
        current_vr_quat: np.ndarray,
        init_vr_quat: np.ndarray,
        init_arm_quat: np.ndarray,
    ) -> np.ndarray:
        """
        变换旋转四元数（支持可配置的正负号调整）

        根据 rotation_preset 的 signs 参数决定使用的方法：
        - signs = [1, 1, 1]: 使用标准相似变换（HMI 方法）
        - signs != [1, 1, 1]: 使用旋转向量变换 + 正负号调整

        参数:
            current_vr_quat: 当前 VR 四元数 [x, y, z, w] (scipy 格式)
            init_vr_quat: 初始 VR 四元数 [x, y, z, w]
            init_arm_quat: 初始机械臂四元数 [x, y, z, w]

        返回:
            目标机械臂四元数 [x, y, z, w]
        """
        # 检查输入有效性
        if np.allclose(current_vr_quat, 0.0):
            return init_arm_quat.copy()

        # 构造旋转对象
        R_init_arm = R.from_quat(init_arm_quat)
        R_init_vr = R.from_quat(init_vr_quat)
        R_curr_vr = R.from_quat(current_vr_quat)

        # 步骤1: 计算 VR 的相对旋转
        R_rel_vr = R_init_vr.inv() * R_curr_vr

        # 步骤2: 根据 signs 选择变换方法
        if not np.allclose(self._rotation_signs, [1, 1, 1]):
            # 使用旋转向量变换 + 正负号调整
            rotvec = R_rel_vr.as_rotvec()
            # 应用坐标变换矩阵将旋转向量映射到机械臂坐标系
            rotvec_arm = self._rotation_base_matrix @ rotvec
            # 应用正负号调整
            rotvec_arm = rotvec_arm * self._rotation_signs
            R_rel_arm = R.from_rotvec(rotvec_arm)
            R_target = R_rel_arm * R_init_arm
        else:
            # 标准相似变换（HMI 方法）
            # Vt = R_init_arm.inv() * R_coord * R_init_vr
            # R_target = R_coord * R_curr_vr * Vt.inv()
            Vt = R_init_arm.inv() * self._R_coord * R_init_vr
            R_target = self._R_coord * R_curr_vr * Vt.inv()

        return R_target.as_quat()


# ============================================================================
# 逆运动学封装
# ============================================================================

class RealmanIK:
    """
    封装 Realman SDK 逆运动学接口
    
    参照 left_arm.py 的实现
    """
    
    @staticmethod
    def quaternion_to_euler(quat: np.ndarray) -> np.ndarray:
        """
        四元数转欧拉角 (内旋 xyz，匹配 Realman SDK 的 RPY 约定)
        
        参数:
            quat: [x, y, z, w] 格式四元数
        
        返回:
            [rx, ry, rz] 弧度
        """
        rot = R.from_quat(quat)
        # 使用 'xyz' 小写内旋顺序，匹配 Realman SDK 的 RPY (Roll-Pitch-Yaw) 约定
        euler = rot.as_euler('xyz', degrees=False)
        return euler
    
    @staticmethod
    def euler_to_quaternion(euler: np.ndarray) -> np.ndarray:
        """
        欧拉角转四元数 (内旋 xyz，匹配 Realman SDK 的 RPY 约定)
        
        参数:
            euler: [rx, ry, rz] 弧度
        
        返回:
            [x, y, z, w] 格式四元数
        """
        # 使用 'xyz' 小写内旋顺序，匹配 Realman SDK 的 RPY 约定
        rot = R.from_euler('xyz', euler, degrees=False)
        return rot.as_quat()
    
    @staticmethod
    def inverse_kinematics(arm, target_pose: List[float], current_joint_deg: List[float]) -> Optional[np.ndarray]:
        """
        调用 Realman SDK 逆运动学
        
        参数:
            arm: RoboticArm 实例
            target_pose: [x, y, z, rx, ry, rz] 位置(米)，欧拉角(弧度)
            current_joint_deg: 当前关节角度(度)
        
        返回:
            目标关节角度(弧度)，或 None（IK 无解）
        """
        try:
            # 导入 Realman SDK 类型
            from Robotic_Arm.rm_robot_interface import rm_inverse_kinematics_params_t
            
            # 构造 IK 输入
            # 注意：Realman SDK 的 q_pose 位置单位是米，角度单位是弧度
            ik_input = rm_inverse_kinematics_params_t(
                q_in=list(current_joint_deg),
                q_pose=list(target_pose),
                flag=1,  # 使用当前关节作为参考
            )
            
            # 调用 IK
            flag, result_deg = arm.rm_algo_inverse_kinematics(ik_input)
            
            if flag != 0:
                return None
            
            # 转换为弧度
            return np.radians(result_deg)
            
        except ImportError:
            print("[RealmanIK] 警告: 无法导入 Realman SDK，IK 不可用")
            return None
        except Exception as e:
            print(f"[RealmanIK] IK 计算错误: {e}")
            return None
    
    @staticmethod
    def forward_kinematics(arm, joint_rad: np.ndarray) -> Optional[np.ndarray]:
        """
        调用 Realman SDK 正运动学
        
        参数:
            arm: RoboticArm 实例
            joint_rad: 关节角度(弧度)
        
        返回:
            [x, y, z, rx, ry, rz] 位置(米)，欧拉角(弧度)
        """
        try:
            joint_deg = np.degrees(joint_rad).tolist()
            pose = arm.rm_algo_forward_kinematics(joint_deg)
            return np.array(pose, dtype=np.float64)
        except Exception as e:
            print(f"[RealmanIK] FK 计算错误: {e}")
            return None


# ============================================================================
# 滤波器
# ============================================================================

class PoseFilter:
    """
    位姿 EMA 滤波器
    
    对位置和旋转分别进行指数移动平均滤波
    """
    
    def __init__(self, alpha_pos: float = 0.3, alpha_rot: float = 0.3):
        """
        初始化滤波器
        
        参数:
            alpha_pos: 位置平滑系数 (0-1，越小越平滑)
            alpha_rot: 旋转平滑系数 (0-1，越小越平滑)
        """
        self.alpha_pos = alpha_pos
        self.alpha_rot = alpha_rot
        self._last_pos: Optional[np.ndarray] = None
        self._last_quat: Optional[np.ndarray] = None
    
    def reset(self, initial_pos: Optional[np.ndarray] = None, initial_quat: Optional[np.ndarray] = None):
        """重置滤波器状态。

        Args:
            initial_pos: 可选的初始位置 [x, y, z]，用于预填充 filter 历史，避免冷启动跳变
            initial_quat: 可选的初始四元数 [x, y, z, w]，用于预填充 filter 历史
        """
        self._last_pos = np.array(initial_pos, dtype=np.float64).copy() if initial_pos is not None else None
        self._last_quat = np.array(initial_quat, dtype=np.float64).copy() if initial_quat is not None else None
    
    def filter_position(self, position: np.ndarray) -> np.ndarray:
        """
        滤波位置
        
        参数:
            position: [x, y, z]
        
        返回:
            滤波后的位置
        """
        pos = np.array(position, dtype=np.float64)
        
        if self._last_pos is None:
            self._last_pos = pos.copy()
            return pos
        
        # EMA: new = alpha * current + (1-alpha) * last
        filtered = self.alpha_pos * pos + (1 - self.alpha_pos) * self._last_pos
        self._last_pos = filtered.copy()
        return filtered
    
    def filter_quaternion(self, quat: np.ndarray) -> np.ndarray:
        """
        滤波四元数
        
        参数:
            quat: [x, y, z, w]
        
        返回:
            滤波后的四元数（已归一化）
        """
        q = np.array(quat, dtype=np.float64)
        q = QuaternionUtils.normalize(q)
        
        if self._last_quat is None:
            self._last_quat = q.copy()
            return q
        
        # 确保四元数在同一半球
        if np.dot(self._last_quat, q) < 0:
            q = -q
        
        # EMA 滤波
        filtered = self.alpha_rot * q + (1 - self.alpha_rot) * self._last_quat
        filtered = QuaternionUtils.normalize(filtered)
        self._last_quat = filtered.copy()
        return filtered
    
    def filter_pose(self, position: np.ndarray, quat: np.ndarray) -> Tuple[np.ndarray, np.ndarray]:
        """
        同时滤波位置和旋转
        
        返回:
            (filtered_position, filtered_quaternion)
        """
        return self.filter_position(position), self.filter_quaternion(quat)


# ============================================================================
# 四元数工具
# ============================================================================

class QuaternionUtils:
    """四元数工具函数"""
    
    @staticmethod
    def normalize(quat: np.ndarray) -> np.ndarray:
        """归一化四元数"""
        norm = np.linalg.norm(quat)
        if norm < 1e-10:
            return np.array([0.0, 0.0, 0.0, 1.0])
        return quat / norm
    
    @staticmethod
    def slerp(q1: np.ndarray, q2: np.ndarray, t: float) -> np.ndarray:
        """
        四元数球面线性插值
        
        参数:
            q1: 起始四元数 [x, y, z, w]
            q2: 目标四元数 [x, y, z, w]
            t: 插值参数 [0, 1]
        
        返回:
            插值结果四元数
        """
        q1 = QuaternionUtils.normalize(q1)
        q2 = QuaternionUtils.normalize(q2)
        
        dot = np.dot(q1, q2)
        
        # 确保最短路径
        if dot < 0:
            q2 = -q2
            dot = -dot
        
        # 如果非常接近，使用线性插值
        if dot > 0.9995:
            result = q1 + t * (q2 - q1)
            return QuaternionUtils.normalize(result)
        
        # 球面线性插值
        theta = np.arccos(dot)
        sin_theta = np.sin(theta)
        
        s1 = np.sin((1 - t) * theta) / sin_theta
        s2 = np.sin(t * theta) / sin_theta
        
        result = s1 * q1 + s2 * q2
        return QuaternionUtils.normalize(result)
    
    @staticmethod
    def multiply(q1: np.ndarray, q2: np.ndarray) -> np.ndarray:
        """
        四元数乘法 q1 * q2
        
        参数:
            q1, q2: [x, y, z, w] 格式四元数
        
        返回:
            乘积四元数
        """
        x1, y1, z1, w1 = q1
        x2, y2, z2, w2 = q2
        
        return np.array([
            w1*x2 + x1*w2 + y1*z2 - z1*y2,
            w1*y2 - x1*z2 + y1*w2 + z1*x2,
            w1*z2 + x1*y2 - y1*x2 + z1*w2,
            w1*w2 - x1*x2 - y1*y2 - z1*z2,
        ])
    
    @staticmethod
    def inverse(quat: np.ndarray) -> np.ndarray:
        """
        四元数求逆（共轭）
        
        参数:
            quat: [x, y, z, w]
        
        返回:
            逆四元数
        """
        return np.array([-quat[0], -quat[1], -quat[2], quat[3]])


# ============================================================================
# 控制状态
# ============================================================================

@dataclass
class VRControlState:
    """VR 控制状态数据结构"""
    
    is_initialized: bool = False
    is_active: bool = False  # trigger 是否按下
    
    # 机械臂初始位姿（trigger 按下时记录）
    init_arm_pos: np.ndarray = field(default_factory=lambda: np.zeros(3))
    init_arm_quat: np.ndarray = field(default_factory=lambda: np.array([0.0, 0.0, 0.0, 1.0]))
    
    # VR 初始位姿（trigger 按下时记录）
    init_vr_pos: np.ndarray = field(default_factory=lambda: np.zeros(3))
    init_vr_quat: np.ndarray = field(default_factory=lambda: np.array([0.0, 0.0, 0.0, 1.0]))
    
    # 当前 VR 输入
    current_vr_pos: np.ndarray = field(default_factory=lambda: np.zeros(3))
    current_vr_quat: np.ndarray = field(default_factory=lambda: np.array([0.0, 0.0, 0.0, 1.0]))
    current_trigger: float = 0.0
    current_grip: float = 0.0
    
    # 时间戳
    last_update_time: float = 0.0
    
    def reset(self):
        """重置状态"""
        self.is_initialized = False
        self.is_active = False
        self.init_arm_pos = np.zeros(3)
        self.init_arm_quat = np.array([0.0, 0.0, 0.0, 1.0])
        self.init_vr_pos = np.zeros(3)
        self.init_vr_quat = np.array([0.0, 0.0, 0.0, 1.0])
