import sys
import time
import threading
from collections import deque
from pathlib import Path
from typing import Optional, Tuple

import numpy as np
import cv2

MIDDLE_POINT = np.array([333.37, 198.9, 118.21, 233.2, 339.08, 272.64, 175.34, 32.5])
MIDDLE_POINT[0] += 150
MIDDLE_POINT[1] += 30
MIDDLE_POINT[5] += 30
MIDDLE_POINT[6] -= 30
EXO_JOINT_NAMES = [
    "joint_1",
    "joint_2",
    "joint_3",
    "joint_4",
    "joint_5",
    "joint_6",
    "joint_7",
    "gripper",
]

ROBOT_JOINT_NAMES = EXO_JOINT_NAMES

# 14 维状态/动作的完整名称列表（与 RoboCOIN 格式兼容）
ROBOT_STATE_NAMES = [
    "joint_1_rad", "joint_2_rad", "joint_3_rad", "joint_4_rad",
    "joint_5_rad", "joint_6_rad", "joint_7_rad", "gripper_open",
    "eef_pos_x_m", "eef_pos_y_m", "eef_pos_z_m",
    "eef_rot_euler_x_rad", "eef_rot_euler_y_rad", "eef_rot_euler_z_rad",
]


def robot_state_names_for_arms(arm_names: list[str]) -> list[str]:
    if not arm_names:
        return ROBOT_STATE_NAMES
    if len(arm_names) == 1:
        return ROBOT_STATE_NAMES
    return [f"{arm}_{name}" for arm in arm_names for name in ROBOT_STATE_NAMES]


def now_s() -> float:
    return time.time()


def find_repo_root(start: Path) -> Optional[Path]:
    for parent in [start] + list(start.parents):
        if (parent / "Robotic_Arm").is_dir():
            return parent
    return None


def add_repo_paths() -> Path:
    repo_root = find_repo_root(Path(__file__).resolve())
    if repo_root is None:
        raise RuntimeError("未找到包含 Robotic_Arm 的仓库根目录，请确认项目路径是否正确。")
    robotic_arm_dir = repo_root / "Robotic_Arm"
    if not robotic_arm_dir.is_dir():
        raise RuntimeError(f"未找到 Robotic_Arm 目录：{robotic_arm_dir}")
    extra_paths = [
        repo_root,
        repo_root / "RoboCOIN" / "src",
        repo_root / "r3kit",
    ]
    for path in extra_paths:
        if path.exists():
            path_str = str(path)
            if path_str not in sys.path:
                sys.path.insert(0, path_str)
    return repo_root


def format_angle(raw: np.ndarray) -> np.ndarray:
    angle = raw.astype(float).copy()
    for i in range(7):
        if MIDDLE_POINT[i] > 180:
            if angle[i] < MIDDLE_POINT[i] - 180:
                angle[i] = angle[i] + 360 - MIDDLE_POINT[i]
            else:
                angle[i] = angle[i] - MIDDLE_POINT[i]
        else:
            if angle[i] > MIDDLE_POINT[i] + 180:
                angle[i] = angle[i] - 360 - MIDDLE_POINT[i]
            else:
                angle[i] = angle[i] - MIDDLE_POINT[i]
    # 251230新加
    angle[0] = -1 * angle[0]
    angle[1] = -1 * angle[1]
    angle[2] = -1 * angle[2]
    angle[3] = -1 * angle[3]
    angle[4] = -1 * angle[4]
    angle[5] = -1 * angle[5]
    angle[6] = -1 * angle[6]
    # 251230已去除下列三行，转移至middle本身更改
    # angle[0] = angle[0] - 30
    # angle[1] = angle[1] - 30
    # angle[5] = angle[5] - 30
    angle[:7] = angle[:7] / 180.0 * np.pi

    if angle[7] <= 136.6:
        angle[7] += 360
    elif 136.6 < angle[7] < 160:
        angle[7] = 136.6 + 360
    elif 160 <= angle[7] < 189.84:
        angle[7] = 189.84
    angle[7] = (angle[7] - 189.84) / (136.6 + 360 - 189.84) * 1000
    return angle


def exo_to_robot_cmd_deg(exo_rad: np.ndarray) -> np.ndarray:
    cmd = np.degrees(exo_rad[:7]).astype(float)
    for idx in (0, 2, 4, 6):
        if idx < len(cmd):
            # cmd[idx] = -cmd[idx]
            cmd[idx] = cmd[idx]
    return cmd


def gripper_open_from_pos(pos: float) -> float:
    return float(np.clip(pos / 1000.0, 0.0, 1.0))


def depth_to_vis_image(
    depth: np.ndarray,
    mode: str,
    min_val: float,
    max_val: float,
    pct_low: float,
    pct_high: float,
    colormap: int,
) -> np.ndarray:
    depth = depth.astype(np.float32)
    mask = depth > 0
    if mode == "percentile":
        if mask.any():
            low, high = np.percentile(depth[mask], [pct_low, pct_high])
        else:
            low, high = 0.0, 1.0
    else:
        low, high = min_val, max_val

    if high <= low:
        high = low + 1e-6

    depth_norm = np.clip((depth - low) / (high - low), 0.0, 1.0)
    depth_norm[~mask] = 0.0
    depth_u8 = (depth_norm * 255.0).astype(np.uint8)
    depth_bgr = cv2.applyColorMap(depth_u8, colormap)
    depth_rgb = cv2.cvtColor(depth_bgr, cv2.COLOR_BGR2RGB)
    return depth_rgb


class TimedBuffer:
    def __init__(self, maxlen: int = 2000):
        self._buf = deque(maxlen=maxlen)
        self._lock = threading.Lock()

    def add(self, ts: float, value: np.ndarray) -> None:
        with self._lock:
            self._buf.append((ts, value))

    def get_nearest(self, ts: float, max_dt: float) -> Optional[Tuple[np.ndarray, float, float]]:
        with self._lock:
            if not self._buf:
                return None
            best_ts, best_val = min(self._buf, key=lambda item: abs(item[0] - ts))
            dt = abs(best_ts - ts)
        if dt > max_dt:
            return None
        return best_val, best_ts, dt
