import threading
import time
import traceback
import queue
import copy
from pathlib import Path
from typing import Dict, Optional, Tuple

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from sensor_msgs.msg import Image, JointState
from std_msgs.msg import String
from std_srvs.srv import Trigger
from cv_bridge import CvBridge
import message_filters

from .utils import (
    ROBOT_JOINT_NAMES,
    ROBOT_STATE_NAMES,
    add_repo_paths,
    robot_state_names_for_arms,
    depth_to_vis_image,
)
from .diag_log import create_diag_logger

_diag_log = create_diag_logger("dataset_recorder")

add_repo_paths()

try:
    from lerobot.datasets.lerobot_dataset import LeRobotDataset
    from lerobot.datasets import utils as ds_utils
    from lerobot.datasets.utils import serialize_dict, append_jsonlines
except ImportError as exc:
    raise RuntimeError("未找到 lerobot，请确认已安装或已加入 PYTHONPATH。") from exc


# ===== 深度统计量优化：聚合为 scalar 并单独存文件 =====

def _is_depth_key(key: str) -> bool:
    """判断是否为深度图 key（排除 depth_vis）"""
    return "_depth" in key and "_depth_vis" not in key


def _aggregate_depth_stats(episode_stats: dict, features: dict) -> tuple[dict, dict]:
    """
    将深度图统计量从 (H, W) 聚合为 scalar。

    Returns:
        (aggregated_stats, depth_stats): 聚合后的统计量（不含深度）, 原始深度统计量
    """
    aggregated = {}
    depth_stats = {}
    for key, value in episode_stats.items():
        if _is_depth_key(key) and features.get(key, {}).get("dtype") == "uint8":
            # 深度图：保留原始统计量到单独文件，主文件中存聚合后的 scalar
            depth_stats[key] = value
            aggregated[key] = {
                "min": np.array([float(np.min(value["min"]))]),
                "max": np.array([float(np.max(value["max"]))]),
                "mean": np.array([float(np.mean(value["mean"]))]),
                "std": np.array([float(np.mean(value["std"]))]),  # 近似：取 std 的均值
                "count": value["count"],
            }
        else:
            aggregated[key] = value
    return aggregated, depth_stats


# 保存原始函数引用
_original_write_episode_stats = ds_utils.write_episode_stats

# 全局变量：当前 dataset 的 features（由 DatasetRecorderNode 设置）
_current_dataset_features: dict = {}


def _patched_write_episode_stats(episode_index: int, episode_stats: dict, local_dir: Path):
    """
    Hook write_episode_stats：
    1. 将深度统计量聚合为 scalar
    2. 深度统计量单独存到 episodes_stats_depth.jsonl
    3. 主文件只存聚合后的统计量
    """
    global _current_dataset_features

    aggregated, depth_stats = _aggregate_depth_stats(episode_stats, _current_dataset_features)

    # 写入主文件（聚合后，不含逐像素深度统计量）
    _original_write_episode_stats(episode_index, aggregated, local_dir)

    # 写入深度统计量到单独文件
    if depth_stats:
        depth_entry = {"episode_index": episode_index, "stats": serialize_dict(depth_stats)}
        append_jsonlines(depth_entry, local_dir / "meta" / "episodes_stats_depth.jsonl")


# 应用 monkey-patch
ds_utils.write_episode_stats = _patched_write_episode_stats


def get_colormap(name: str) -> int:
    name = name.lower()
    mapping = {
        "jet": cv2.COLORMAP_JET,
        "turbo": cv2.COLORMAP_TURBO,
        "viridis": cv2.COLORMAP_VIRIDIS,
        "plasma": cv2.COLORMAP_PLASMA,
        "magma": cv2.COLORMAP_MAGMA,
        "inferno": cv2.COLORMAP_INFERNO,
    }
    if name not in mapping:
        raise ValueError(f"不支持的颜色映射: {name}")
    return mapping[name]


def build_features(
    camera_shapes: Dict[str, Tuple[int, int, int]],
    depth_shapes: Dict[str, Tuple[int, int]],
    use_depth: bool,
    depth_vis: bool,
    depth_scale_map: Dict[str, Optional[float]],
    depth_max_m: float,
    state_names=None,
):
    # state/action: 每臂 14 维，双臂时按 state_topics 顺序拼接。
    state_names = list(state_names) if state_names else ROBOT_STATE_NAMES
    state_shape = (len(state_names),)
    features = {
        "observation.state": {"dtype": "float32", "shape": state_shape, "names": state_names},
        "action": {"dtype": "float32", "shape": state_shape, "names": state_names},
    }
    for cam_name, shape in camera_shapes.items():
        height, width, channels = shape
        features[f"observation.images.{cam_name}_rgb"] = {
            "dtype": "video",
            "shape": [height, width, channels],
            "names": ["height", "width", "channels"],
        }
        if use_depth and cam_name in depth_shapes:
            depth_h, depth_w = depth_shapes[cam_name]
            depth_info = {
                "depth.max_m": float(depth_max_m),
                "depth.u8_scale_m": float(depth_max_m) / 255.0,
            }
            scale = depth_scale_map.get(cam_name)
            if scale is not None:
                depth_info["depth.scale_m_per_raw"] = float(scale)
            features[f"observation.images.{cam_name}_depth"] = {
                "dtype": "uint8",
                "shape": (depth_h, depth_w),
                "names": ["height", "width"],
                "info": depth_info,
            }
            if depth_vis:
                features[f"observation.images.{cam_name}_depth_vis"] = {
                    "dtype": "video",
                    "shape": [depth_h, depth_w, 3],
                    "names": ["height", "width", "channels"],
                }
    return features


class DatasetRecorderNode(Node):
    def __init__(self):
        super().__init__("dataset_recorder_node")
        self.declare_parameter("repo_id", "local/realman_teleop")
        self.declare_parameter("dataset_root", "/tmp/lerobot_dataset")
        self.declare_parameter("task", "pick_and_place")
        self.declare_parameter("record_hz", 30.0)
        self.declare_parameter("sync_max_dt", 0.05)
        self.declare_parameter("sync_queue_size", 30)
        self.declare_parameter("cam_sync_max_dt", 0.03)
        self.declare_parameter("camera_names", ["cam0"])
        self.declare_parameter("color_topics", Parameter.Type.STRING_ARRAY)
        self.declare_parameter("depth_topics", Parameter.Type.STRING_ARRAY)
        self.declare_parameter("color_encoding", "rgb8")
        self.declare_parameter("use_depth", True)
        self.declare_parameter("depth_vis", True)
        self.declare_parameter("depth_vis_mode", "fixed")
        self.declare_parameter("depth_vis_min", 0.0)
        self.declare_parameter("depth_vis_max", 1.5)
        self.declare_parameter("depth_vis_pct_low", 1.0)
        self.declare_parameter("depth_vis_pct_high", 99.0)
        self.declare_parameter("depth_vis_colormap", "turbo")
        self.declare_parameter("depth_vis_use_meters", True)
        self.declare_parameter("depth_scales", Parameter.Type.DOUBLE_ARRAY)
        self.declare_parameter("depth_max_m", 2.0)
        # 废弃参数：VR 模式不再订阅 action 话题，action 由后处理生成（action[t] = state[t+1]）
        self.declare_parameter("action_topic", "/rm/action_joint_state")
        self.declare_parameter("state_topic", "/rm/state_joint_state")
        self.declare_parameter("state_topics", Parameter.Type.STRING_ARRAY)
        self.declare_parameter("arm_names", Parameter.Type.STRING_ARRAY)
        self.declare_parameter("dataset_ready_timeout", 30.0)
        self.declare_parameter("mock_state", False)
        self.declare_parameter("mock_state_mode", "action")
        self.declare_parameter("image_writer_threads", 4)
        self.declare_parameter("image_writer_processes", 0)
        self.declare_parameter("timestamp_tolerance_s", 0.04)

        self.repo_id = self.get_parameter("repo_id").get_parameter_value().string_value
        self.dataset_root = self.get_parameter("dataset_root").get_parameter_value().string_value
        self.task = self.get_parameter("task").get_parameter_value().string_value
        self.record_hz = self.get_parameter("record_hz").get_parameter_value().double_value
        self.sync_max_dt = self.get_parameter("sync_max_dt").get_parameter_value().double_value
        self.sync_queue_size = self.get_parameter("sync_queue_size").get_parameter_value().integer_value
        self.cam_sync_max_dt = self.get_parameter("cam_sync_max_dt").get_parameter_value().double_value
        self.camera_names = list(self.get_parameter("camera_names").value)
        self.color_topics = list(self.get_parameter("color_topics").value)
        self.depth_topics = list(self.get_parameter("depth_topics").value)
        self.color_encoding = self.get_parameter("color_encoding").get_parameter_value().string_value
        self.use_depth = self.get_parameter("use_depth").get_parameter_value().bool_value
        self.depth_vis = self.get_parameter("depth_vis").get_parameter_value().bool_value
        self.depth_vis_mode = self.get_parameter("depth_vis_mode").get_parameter_value().string_value
        self.depth_vis_min = self.get_parameter("depth_vis_min").get_parameter_value().double_value
        self.depth_vis_max = self.get_parameter("depth_vis_max").get_parameter_value().double_value
        self.depth_vis_pct_low = self.get_parameter("depth_vis_pct_low").get_parameter_value().double_value
        self.depth_vis_pct_high = self.get_parameter("depth_vis_pct_high").get_parameter_value().double_value
        self.depth_vis_colormap = self.get_parameter("depth_vis_colormap").get_parameter_value().string_value
        self.depth_vis_use_meters = self.get_parameter("depth_vis_use_meters").get_parameter_value().bool_value
        self.depth_scales = list(self.get_parameter("depth_scales").value)
        self.depth_max_m = self.get_parameter("depth_max_m").get_parameter_value().double_value
        self.action_topic = self.get_parameter("action_topic").get_parameter_value().string_value
        self.state_topic = self.get_parameter("state_topic").get_parameter_value().string_value
        self.state_topics = list(self.get_parameter("state_topics").value)
        if not self.state_topics:
            self.state_topics = [self.state_topic]
        self.arm_names = list(self.get_parameter("arm_names").value)
        if not self.arm_names:
            self.arm_names = ["arm"] if len(self.state_topics) == 1 else [f"arm{i}" for i in range(len(self.state_topics))]
        if len(self.arm_names) != len(self.state_topics):
            raise ValueError("arm_names 的数量必须与 state_topics 一致。")
        self.state_names = robot_state_names_for_arms(self.arm_names)
        self.state_dim = len(self.state_names)
        self.dataset_ready_timeout = (
            self.get_parameter("dataset_ready_timeout").get_parameter_value().double_value
        )
        self.mock_state = self.get_parameter("mock_state").get_parameter_value().bool_value
        self.mock_state_mode = self.get_parameter("mock_state_mode").get_parameter_value().string_value
        
        # 支持通过 dry_run 参数自动控制 mock_state
        # 在 launch 文件中可以将 driver 的 dry_run 参数同时传给 recorder
        self.declare_parameter("dry_run", False)
        dry_run = self.get_parameter("dry_run").get_parameter_value().bool_value
        if dry_run and not self.mock_state:
            self.mock_state = True
            self.get_logger().info("检测到 dry_run=true，自动启用 mock_state")
        
        self.image_writer_threads = self.get_parameter("image_writer_threads").get_parameter_value().integer_value
        self.image_writer_processes = self.get_parameter("image_writer_processes").get_parameter_value().integer_value
        self.timestamp_tolerance_s = (
            self.get_parameter("timestamp_tolerance_s").get_parameter_value().double_value
        )

        if self.color_encoding not in ("rgb8", "bgr8"):
            raise ValueError("color_encoding 仅支持 rgb8 或 bgr8。")

        if not self.color_topics:
            self.color_topics = [f"/camera/{name}/color/image_raw" for name in self.camera_names]
        if not self.depth_topics:
            self.depth_topics = [f"/camera/{name}/depth/image_raw" for name in self.camera_names]

        if len(self.color_topics) != len(self.camera_names):
            raise ValueError("color_topics 的数量必须与 camera_names 一致。")
        if self.use_depth and len(self.depth_topics) != len(self.camera_names):
            raise ValueError("depth_topics 的数量必须与 camera_names 一致。")

        self.depth_scale_map = {}
        for idx, name in enumerate(self.camera_names):
            scale = None
            if idx < len(self.depth_scales):
                scale = self.depth_scales[idx]
            self.depth_scale_map[name] = scale

        if self.use_depth:
            for name in self.camera_names:
                scale = self.depth_scale_map.get(name)
                if scale is None or scale <= 0:
                    raise RuntimeError(f"相机 {name} 缺少有效的 depth_scale，请在 depth_scales 中配置。")

        if self.depth_max_m <= 0:
            raise ValueError("depth_max_m 必须为正数。")

        if self.timestamp_tolerance_s < 0:
            raise ValueError("timestamp_tolerance_s 不能为负数。")

        if self.sync_queue_size <= 0:
            raise ValueError("sync_queue_size 必须为正整数。")

        self.bridge = CvBridge()
        self.depth_vis_colormap = get_colormap(self.depth_vis_colormap)
        # 节流系数 0.5：只过滤 >60Hz 的重复帧，不误杀正常 30Hz 帧（参考 DAgger PITFALL-021）
        self.min_frame_interval = 0.5 / self.record_hz if self.record_hz > 0 else 0.0

        # ===== 录制诊断计数器（参考 DAgger REC-STATS） =====
        self._sync_triggered = 0      # ApproximateTimeSynchronizer 触发次数
        self._sync_throttled = 0      # 被节流丢弃的帧数
        self._sync_recorded = 0       # 实际录制的帧数
        self._sync_stats_last_time = 0.0

        self.dataset: Optional[LeRobotDataset] = None
        self.dataset_lock = threading.Lock()
        self.dataset_ready = threading.Event()
        self.state_lock = threading.Lock()
        self.recording = False
        self.current_episode = -1
        self.episode_start_ts = 0.0
        self.last_write_ts: Optional[float] = None

        # 不再订阅 action 话题，action 将在后处理阶段生成 (action[t] = state[t+1])
        self.action_sub = None
        self.state_subs = []
        if not self.mock_state:
            self.state_subs = [
                message_filters.Subscriber(self, JointState, topic, qos_profile=10)
                for topic in self.state_topics
            ]

        self.color_subs = [
            message_filters.Subscriber(self, Image, topic, qos_profile=10) for topic in self.color_topics
        ]
        self.depth_subs = []
        if self.use_depth:
            self.depth_subs = [
                message_filters.Subscriber(self, Image, topic, qos_profile=10) for topic in self.depth_topics
            ]

        # 改进后：以 state 为同步基准，不再依赖 action 话题
        sync_inputs = []
        if self.state_subs:
            sync_inputs.extend(self.state_subs)
        elif not self.mock_state:
            raise RuntimeError("VR 模式必须有 state 话题数据，请确保 realman_driver_node 已启动。")
        # 注意：mock_state=True 时无 state 订阅，仅使用相机同步（state 将被 mock）
        sync_inputs.extend(self.color_subs)
        if self.use_depth:
            sync_inputs.extend(self.depth_subs)

        self.sync = message_filters.ApproximateTimeSynchronizer(
            sync_inputs,
            self.sync_queue_size,
            self.sync_max_dt,
            allow_headerless=False,
        )
        self.sync.registerCallback(self._on_sync)

        # ===== Service 控制 =====
        self.start_srv = self.create_service(Trigger, "/recorder/start", self._handle_start)
        self.stop_srv = self.create_service(Trigger, "/recorder/stop", self._handle_stop)
        self.pause_srv = self.create_service(Trigger, "/recorder/pause", self._handle_pause)
        self.discard_srv = self.create_service(Trigger, "/recorder/discard", self._handle_discard)
        self._episode_start_event = threading.Event()
        self._episode_stop_event = threading.Event()
        self._episode_discard_flag = False  # 是否丢弃当前 episode
        
        # ===== 状态发布（供 UI 监控） =====
        # 状态: initializing / waiting / recording / confirming / saving / ready / error
        self._recorder_status = "initializing"
        self._recorder_frame_count = 0
        self._error_message = ""  # 错误信息，供 UI 显示
        self._status_pub = self.create_publisher(String, "/recorder/status", 10)
        self._status_timer = self.create_timer(0.1, self._publish_status)  # 10Hz 发布状态

        # ===== 异步存包队列 =====
        self._save_queue = queue.Queue(maxsize=20)  # 最多缓存20个待存包的episode
        self._current_saving_episode = None  # 当前正在存包的episode编号

        self.stop_event = threading.Event()

        self._save_thread = threading.Thread(target=self._save_loop, daemon=True)
        self._save_thread.start()

        self.record_thread = threading.Thread(target=self._record_loop, daemon=True)
        self.record_thread.start()

    def _ros_stamp_to_time(self, stamp) -> float:
        ts = float(stamp.sec) + float(stamp.nanosec) * 1e-9
        if ts <= 0:
            ts = time.time()
        return ts
    
    def _publish_status(self):
        """定时发布录制状态（JSON 格式）。"""
        import json
        with self.state_lock:
            duration = 0.0
            if self.recording and self.episode_start_ts > 0:
                duration = time.time() - self.episode_start_ts

            # 计算存包队列状态
            queue_size = self._save_queue.qsize()
            saving_episode = self._current_saving_episode

            status_data = {
                "status": self._recorder_status,
                "episode": self.current_episode,
                "frame_count": self._recorder_frame_count,
                "recording": self.recording,
                "duration": round(duration, 1),
                "error_message": self._error_message,
                "saving_episode": saving_episode,      # 新增：当前存包的episode
                "save_queue_size": queue_size,         # 新增：存包队列长度
            }
        msg = String()
        msg.data = json.dumps(status_data)
        self._status_pub.publish(msg)

    def _split_sync_msgs(self, msgs):
        """解析同步消息，返回 (state_msgs, color_msgs, depth_msgs)。

        注意：action 不再从话题订阅，将在 save_episode 时后处理生成。
        """
        state_count = 0 if self.mock_state else len(self.state_topics)
        expected = state_count + len(self.camera_names)
        if self.use_depth:
            expected += len(self.camera_names)
        if len(msgs) != expected:
            raise RuntimeError(f"同步消息数量不匹配：期望 {expected}，实际 {len(msgs)}。")

        index = 0
        state_msgs = []
        if state_count:
            state_msgs = list(msgs[index : index + state_count])
            index += state_count
        color_msgs = list(msgs[index : index + len(self.camera_names)])
        index += len(self.camera_names)
        depth_msgs = []
        if self.use_depth:
            depth_msgs = list(msgs[index : index + len(self.camera_names)])
        return state_msgs, color_msgs, depth_msgs

    def _convert_color_msgs(self, color_msgs):
        frames = {}
        for name, msg in zip(self.camera_names, color_msgs):
            frames[name] = self.bridge.imgmsg_to_cv2(msg, desired_encoding=self.color_encoding)
        return frames

    def _convert_depth_msgs(self, depth_msgs):
        frames = {}
        for name, msg in zip(self.camera_names, depth_msgs):
            frames[name] = self.bridge.imgmsg_to_cv2(msg, desired_encoding="16UC1")
        return frames

    def _ensure_dataset(self, color_frames, depth_frames) -> None:
        if self.dataset is not None:
            return
        camera_shapes = {name: frame.shape for name, frame in color_frames.items()}
        depth_shapes = {name: frame.shape for name, frame in depth_frames.items()}
        self.dataset = self._build_dataset(camera_shapes, depth_shapes)
        self.dataset_ready.set()

    def _fatal_error(self, message: str, exc: Optional[Exception] = None) -> None:
        # 更新状态为 error，让 UI 知道出错了
        with self.state_lock:
            self._recorder_status = "error"
            self._error_message = message if exc is None else f"{message}: {exc}"
        # 手动发布一次状态，确保 UI 能收到
        self._publish_status()
        time.sleep(0.5)  # 等待状态发布完成
        if exc is None:
            self.get_logger().error(message)
            rclpy.shutdown()
            raise RuntimeError(message)
        self.get_logger().error(f"{message}: {exc}")
        rclpy.shutdown()
        raise RuntimeError(message) from exc

    def _wait_for_dataset_ready(self) -> None:
        timeout = float(self.dataset_ready_timeout)
        if timeout <= 0:
            while not self.dataset_ready.wait(timeout=1.0):
                self.get_logger().warning("尚未收到同步消息，继续等待...")
            return
        start = time.time()
        last_warn = 0.0
        while not self.dataset_ready.wait(timeout=1.0):
            elapsed = time.time() - start
            if elapsed >= timeout:
                self._fatal_error("未收到同步消息，录制线程退出。")
            if elapsed - last_warn >= 5.0:
                self.get_logger().warning(f"尚未收到同步消息，已等待 {elapsed:.1f} 秒。")
                last_warn = elapsed

    def _build_dataset(
        self,
        camera_shapes: Dict[str, Tuple[int, int, int]],
        depth_shapes: Dict[str, Tuple[int, int]],
    ) -> LeRobotDataset:
        dataset_root = Path(self.dataset_root)
        dataset_root.mkdir(parents=True, exist_ok=True)
        dataset_dir = dataset_root / self.repo_id
        if dataset_dir.exists():
            raise FileExistsError(f"数据集目录已存在：{dataset_dir}")

        if self.use_depth and len(depth_shapes) != len(self.camera_names):
            raise ValueError("已开启深度，但部分相机没有深度帧。")
        if self.depth_vis and not self.use_depth:
            raise ValueError("depth_vis 依赖 use_depth，请先开启 use_depth。")

        features = build_features(
            camera_shapes,
            depth_shapes,
            self.use_depth,
            self.depth_vis,
            self.depth_scale_map,
            self.depth_max_m,
            self.state_names,
        )
        # 设置全局 features，供 write_episode_stats hook 使用
        global _current_dataset_features
        _current_dataset_features = features

        return LeRobotDataset.create(
            repo_id=self.repo_id,
            fps=int(self.record_hz),
            root=dataset_dir,
            robot_type="realman",
            features=features,
            use_videos=True,
            tolerance_s=float(self.timestamp_tolerance_s),
            image_writer_threads=self.image_writer_threads,
            image_writer_processes=self.image_writer_processes,
        )

    def _on_sync(self, *msgs):
        if self.stop_event.is_set():
            return
        self._sync_triggered += 1
        try:
            # 改为三元组返回（无 action_msg）
            state_msgs, color_msgs, depth_msgs = self._split_sync_msgs(msgs)
        except Exception as exc:
            self._fatal_error("同步消息解析失败", exc)

        # 验证 state 消息（不再依赖 action）
        if state_msgs:
            parts = []
            for state_msg in state_msgs:
                if not state_msg.position or len(state_msg.position) < 14:
                    return
                parts.append(np.array(state_msg.position[:14], dtype=np.float32))
            state_vec = np.concatenate(parts).astype(np.float32)
        elif self.mock_state:
            state_vec = np.zeros(self.state_dim, dtype=np.float32)
        else:
            return

        color_ts_list = [self._ros_stamp_to_time(msg.header.stamp) for msg in color_msgs]
        if not color_ts_list:
            return
        cam_skew = max(color_ts_list) - min(color_ts_list)
        if cam_skew > self.cam_sync_max_dt:
            return

        t_cam = max(color_ts_list)
        # 移除 action 时间戳检查（不再订阅 action 话题）
        for state_msg in state_msgs:
            state_ts = self._ros_stamp_to_time(state_msg.header.stamp)
            if abs(state_ts - t_cam) > self.sync_max_dt:
                return

        with self.state_lock:
            recording = self.recording
            last_write_ts = self.last_write_ts
        need_dataset = self.dataset is None
        if not recording and not need_dataset:
            return

        color_frames = self._convert_color_msgs(color_msgs)
        depth_frames = self._convert_depth_msgs(depth_msgs) if self.use_depth else {}

        if need_dataset:
            try:
                self._ensure_dataset(color_frames, depth_frames)
            except Exception as exc:
                self._fatal_error("数据集初始化失败", exc)
            if not recording:
                return

        if self.min_frame_interval > 0 and last_write_ts is not None:
            if t_cam - last_write_ts < self.min_frame_interval:
                self._sync_throttled += 1
                self._log_recording_stats()
                return
        if last_write_ts is not None and t_cam <= last_write_ts:
            self._sync_throttled += 1
            self._log_recording_stats()
            return

        # 构建帧数据（action 暂时用 state 占位，save_episode 时后处理替换为 state[t+1]）
        frame = {
            "observation.state": state_vec.astype(np.float32),
            "action": state_vec.astype(np.float32),  # 临时占位
        }
        for name, color in color_frames.items():
            frame[f"observation.images.{name}_rgb"] = color
        if self.use_depth:
            for name, depth in depth_frames.items():
                scale = self.depth_scale_map.get(name)
                if scale is None or scale <= 0:
                    raise RuntimeError(f"相机 {name} 缺少有效的 depth_scale，无法转换深度。")
                depth_m_full = depth.astype(np.float32) * float(scale)
                depth_m_clip = np.clip(depth_m_full, 0.0, float(self.depth_max_m))
                depth_u8 = np.rint(depth_m_clip * (255.0 / float(self.depth_max_m))).astype(np.uint8)
                frame[f"observation.images.{name}_depth"] = depth_u8
                if self.depth_vis:
                    depth_vis_data = depth
                    if self.depth_vis_use_meters:
                        depth_vis_data = depth_m_full
                    frame[f"observation.images.{name}_depth_vis"] = depth_to_vis_image(
                        depth_vis_data,
                        mode=self.depth_vis_mode,
                        min_val=self.depth_vis_min,
                        max_val=self.depth_vis_max,
                        pct_low=self.depth_vis_pct_low,
                        pct_high=self.depth_vis_pct_high,
                        colormap=self.depth_vis_colormap,
                    )

        with self.dataset_lock:
            if not self.recording or self.dataset is None:
                return
            self.dataset.add_frame(frame, task=self.task)
        with self.state_lock:
            self.last_write_ts = t_cam
            self._recorder_frame_count += 1
        self._sync_recorded += 1
        self._log_recording_stats()

    def _log_recording_stats(self):
        """每 10 秒输出一次录制诊断统计（参考 DAgger REC-STATS）。"""
        now = time.time()
        if self._sync_stats_last_time == 0.0:
            self._sync_stats_last_time = now
            return
        elapsed = now - self._sync_stats_last_time
        if elapsed < 10.0:
            return
        fps = self._sync_recorded / elapsed if elapsed > 0 else 0.0
        self.get_logger().info(
            f"[REC-STATS] {elapsed:.1f}s: "
            f"triggered={self._sync_triggered} "
            f"throttled={self._sync_throttled} "
            f"recorded={self._sync_recorded} "
            f"fps={fps:.1f}"
        )
        # 重置计数器
        self._sync_triggered = 0
        self._sync_throttled = 0
        self._sync_recorded = 0
        self._sync_stats_last_time = now

    def _handle_start(self, request, response):
        """Service 回调：开始录制当前 episode。"""
        with self.state_lock:
            status = self._recorder_status
        if status != "waiting":
            response.success = False
            response.message = f"当前状态为 {status}，尚不能开始录制"
            return response
        self._episode_start_event.set()
        response.success = True
        response.message = "开始录制信号已发送"
        return response

    def _handle_pause(self, request, response):
        """Service 回调：暂停录制，进入确认状态（等待用户选择保存或丢弃）。"""
        with self.state_lock:
            if self._recorder_status != "recording":
                response.success = False
                response.message = "当前未在录制中"
                return response
            self.recording = False
            self._recorder_status = "confirming"
        response.success = True
        response.message = "录制已暂停，等待确认"
        return response

    def _handle_stop(self, request, response):
        """Service 回调：停止录制当前 episode（保存）。"""
        self._episode_discard_flag = False
        self._episode_stop_event.set()
        response.success = True
        response.message = "停止录制，准备保存"
        return response

    def _handle_discard(self, request, response):
        """Service 回调：停止录制并丢弃当前 episode（不保存）。"""
        self._episode_discard_flag = True
        self._episode_stop_event.set()
        response.success = True
        response.message = "停止录制，丢弃当前回合"
        return response

    def _postprocess_episode_actions(self) -> None:
        """
        后处理 episode buffer 中的 action 字段。

        将 action[t] 设置为 state[t+1]，使得录制的数据满足模仿学习的格式：
        - 给定 state[t]，模型需要预测 action[t]（即下一时刻的状态 state[t+1]）

        对于最后一帧 T-1，action[T-1] = state[T-1]（保持不变）。
        """
        if self.dataset is None:
            return

        buffer = self.dataset.episode_buffer
        if buffer is None:
            return

        size = buffer.get("size", 0)
        if size <= 1:
            self.get_logger().warning("episode 帧数不足，无法进行 action 后处理。")
            return

        states = buffer.get("observation.state")
        actions = buffer.get("action")

        if states is None or actions is None:
            self.get_logger().error("episode buffer 中缺少 state 或 action 字段。")
            return

        # action[t] = state[t+1]，最后一帧保持不变
        for t in range(size - 1):
            actions[t] = states[t + 1].copy()
        # 最后一帧 action 保持为当前 state（已在录制时设置）

        self.get_logger().info(f"action 后处理完成：{size} 帧，action[t] = state[t+1]")

    def _record_loop(self):
        self._wait_for_dataset_ready()
        self._episode_start_event.clear()

        ep_idx = 0
        while not self.stop_event.is_set():
            # Keep recording indexes independent from asynchronously persisted metadata.
            with self.dataset_lock:
                if self.dataset is None:
                    return
                buffer = self.dataset.episode_buffer
                buffer_index = None if buffer is None else int(buffer["episode_index"])
                if buffer_index != ep_idx:
                    self.dataset.episode_buffer = self.dataset.create_episode_buffer(
                        episode_index=ep_idx
                    )

            # 等待用户点击开始
            with self.state_lock:
                self._recorder_status = "waiting"
                self.current_episode = ep_idx
            self.get_logger().info(f"等待开始第 {ep_idx} 条回合... (调用 /recorder/start)")
            while not self._episode_start_event.wait(timeout=1.0):
                if self.stop_event.is_set():
                    return
            self._episode_start_event.clear()

            self.get_logger().info(f"开始录制第 {ep_idx} 条回合...")
            with self.state_lock:
                self.recording = True
                self.current_episode = ep_idx
                self.episode_start_ts = time.time()
                self.last_write_ts = None
                self._recorder_status = "recording"
                self._recorder_frame_count = 0
            
            # 等待用户点击停止（可能先进入 confirming 状态）
            self._episode_stop_event.clear()
            while not self.stop_event.is_set():
                if self._episode_stop_event.wait(timeout=0.02):
                    break

            with self.state_lock:
                self.recording = False
                self._recorder_status = "saving"

            # 检查是否丢弃
            if self._episode_discard_flag:
                with self.dataset_lock:
                    if self.dataset is not None and hasattr(self.dataset, 'episode_buffer'):
                        # 清空 episode buffer，不保存
                        self.dataset.episode_buffer = None
                self.get_logger().warn(f"第 {ep_idx} 条回合已丢弃（用户选择不保存）")
                self._episode_discard_flag = False
            else:
                with self.dataset_lock:
                    if self.dataset is None:
                        self.get_logger().warning("数据集尚未初始化，已跳过保存。")
                    elif self.dataset.episode_buffer and self.dataset.episode_buffer.get("size", 0) > 0:
                        # 后处理：action[t] = state[t+1]
                        self._postprocess_episode_actions()

                        # 等待当前 episode 的图片全部写入磁盘，
                        # 避免 _save_thread 中 _wait_image_writer 与新 episode 的 add_frame 竞争
                        if self.dataset.image_writer is not None:
                            _diag_log(f"[QUEUE] 等待 image_writer 完成...")
                            self.dataset.image_writer.wait_until_done()
                            _diag_log(f"[QUEUE] image_writer 已完成")

                        # 有序异步存包：LeRobot 的图片目录在 add_frame() 时按 episode_index 生成，
                        # save_episode() 也必须用同一个 index 编码并删除目录。这里只拷贝并入队，
                        # 后台保存线程按 FIFO 顺序落盘，绝不重写 buffer["episode_index"]。
                        current_index = int(self.dataset.episode_buffer["episode_index"])
                        save_task = {
                            "episode": ep_idx,
                            "buffer": copy.deepcopy(self.dataset.episode_buffer),
                        }
                        try:
                            self._save_queue.put_nowait(save_task)
                            _diag_log(
                                f"[QUEUE] 第 {ep_idx} 条回合已加入存包队列"
                                f"（index={current_index}, size={save_task['buffer'].get('size', '?')}）"
                            )
                            self.get_logger().info(f"第 {ep_idx} 条回合已加入存包队列")
                        except queue.Full:
                            self.get_logger().error("存包队列已满，请等待当前存包完成")
                            with self.state_lock:
                                self._recorder_status = "error"
                                self._error_message = "存包队列已满"
                            return

                        ep_idx = current_index + 1
                        self.dataset.episode_buffer = self.dataset.create_episode_buffer(
                            episode_index=ep_idx
                        )
                    else:
                        self.get_logger().warning("当前回合未写入任何帧，已跳过保存。")
            
            self._episode_start_event.clear()

    def _save_loop(self):
        """异步存包循环，在后台线程中执行。"""
        _diag_log("[SAVE-LOOP] 存包线程已启动")
        try:
            while not self.stop_event.is_set():
                try:
                    save_task = self._save_queue.get(timeout=1.0)
                except queue.Empty:
                    continue
                if not self._save_one(save_task):
                    self.stop_event.set()
                    break

            # stop_event 已设置，排空队列中剩余任务
            remaining = 0
            drain_ok = True
            while True:
                try:
                    save_task = self._save_queue.get_nowait()
                except queue.Empty:
                    break
                remaining += 1
                _diag_log(f"[SAVE-LOOP] [关闭] 正在保存队列中剩余的 episode（剩余 {remaining}）...")
                self.get_logger().info(f"[关闭] 正在保存队列中剩余的 episode（剩余 {remaining}）...")
                if not self._save_one(save_task):
                    drain_ok = False
                    break
            if remaining > 0 and drain_ok:
                _diag_log(f"[SAVE-LOOP] [关闭] 队列中 {remaining} 个 episode 已全部保存完成。")
                self.get_logger().info(f"[关闭] 队列中 {remaining} 个 episode 已全部保存完成。")
        except Exception:
            tb = traceback.format_exc()
            _diag_log(f"[SAVE-LOOP] 存包线程异常退出:\n{tb}")
        _diag_log("[SAVE-LOOP] 存包线程已退出")

    def _save_one(self, save_task) -> bool:
        """保存单个 episode。

        LeRobot 在 add_frame() 时已经按 episode_index 写入图片目录，
        save_episode() 又要求 episode_index == meta.total_episodes。这里不能为了通过
        校验而重写 buffer["episode_index"]，否则会编码/删除错误的图片目录。
        """
        episode_num = save_task["episode"]
        buffer = save_task["buffer"]
        buffer_index = int(buffer["episode_index"])
        expected_index = int(self.dataset.meta.total_episodes)

        _diag_log(f"[SAVE] 开始存包 Episode {episode_num}（index={buffer_index}）")
        _diag_log(f"[SAVE] buffer keys: {list(buffer.keys())}")
        _diag_log(f"[SAVE] buffer size: {buffer.get('size', 'N/A')}, total_episodes: {expected_index}")
        self.get_logger().info(f"开始存包 Episode {episode_num}（index={buffer_index}）...")
        with self.state_lock:
            self._current_saving_episode = episode_num

        try:
            if buffer_index != expected_index:
                raise RuntimeError(
                    f"episode_index 不连续：buffer={buffer_index}, "
                    f"meta.total_episodes={expected_index}。为避免覆盖/误删图片目录，已停止保存。"
                )

            start_time = time.time()
            self.dataset.save_episode(buffer)
            elapsed = time.time() - start_time

            _diag_log(f"[SAVE] Episode {episode_num} 存包完成，耗时 {elapsed:.1f}秒")
            self.get_logger().info(f"Episode {episode_num} 存包完成，耗时 {elapsed:.1f}秒")

            with self.state_lock:
                self._current_saving_episode = None
            return True

        except Exception as e:
            tb = traceback.format_exc()
            _diag_log(f"[SAVE-ERROR] Episode {episode_num} 存包失败:\n{tb}")
            self.get_logger().error(f"存包失败: {e}")
            with self.state_lock:
                self._recorder_status = "error"
                self._error_message = f"存包失败: {e}"
                self._current_saving_episode = None
            return False

    def destroy_node(self):
        self.stop_event.set()
        if self.record_thread.is_alive():
            self.record_thread.join(timeout=2.0)

        # 等待保存线程完成队列中剩余的 episode
        remaining = self._save_queue.qsize()
        if remaining > 0:
            self.get_logger().info(f"[关闭] 等待保存线程完成剩余 {remaining} 个 episode...")
        self._save_thread.join(timeout=30.0)
        if self._save_thread.is_alive():
            still_remaining = self._save_queue.qsize()
            self.get_logger().warning(
                f"[关闭] 保存线程等待超时（30秒），仍有 {still_remaining} 个 episode 未保存完成，强制退出。"
            )

        # 清理 LeRobotDataset 的 temp/ PNG 缓存目录（PITFALL-023）
        self._cleanup_image_cache()

        super().destroy_node()

    def _cleanup_image_cache(self):
        """清理 LeRobotDataset 的 temp/ PNG 缓存目录。

        录制时每帧 PNG 先写入 ./temp/{repo_id}/，save_episode() 编码为 MP4 后删除。
        进程异常退出时这些 PNG 不会被清理，会持续占用磁盘空间。
        """
        import shutil
        try:
            dataset = getattr(self, 'dataset', None)
            if dataset is None:
                return
            cache_root = getattr(dataset, 'image_cache_root', None)
            if cache_root is not None and cache_root.exists():
                shutil.rmtree(cache_root)
                self.get_logger().info(f"[关闭] 已清理图像缓存目录: {cache_root}")
        except Exception as e:
            self.get_logger().warning(f"[关闭] 清理图像缓存目录失败: {e}")


def main():
    rclpy.init()
    node = DatasetRecorderNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
