# Realman RM-75 VR 遥操作系统

基于 ROS2 的 VR 遥操作控制 Realman RM-75-6FB 机械臂系统，支持数据采集、Web UI 监控、LeRobot 数据集录制。

## 目录

- [系统架构](#系统架构)
- [节点功能说明](#节点功能说明)
- [通信拓扑图](#通信拓扑图)
- [数据记录格式](#数据记录格式)
- [环境准备](#环境准备)
- [配置说明](#配置说明)
- [使用方法](#使用方法)
  - [一键启动](#一键启动)
  - [分终端启动（开发调试）](#分终端启动开发调试)
- [Web UI 使用](#web-ui-使用)
- [VR 控制映射](#vr-控制映射)
- [常见问题](#常见问题)

---

## 系统架构

```
┌─────────────────────────────────────────────────────────────────────────────┐
│                          ROS2 VR 遥操作节点架构                               │
├─────────────────────────────────────────────────────────────────────────────┤
│                                                                             │
│   ┌──────────────┐                          ┌──────────────┐                │
│   │ VR 头显      │                          │ vr_input_    │                │
│   │ (Quest/Pico) │  ─── ADB/UDP ───────────►│    node      │                │
│   └──────────────┘                          └──────┬───────┘                │
│                                                    │                        │
│                          /vr/right/pose, /vr/right/trigger, /vr/right/grip  │
│                                                    │                        │
│                                                    ▼                        │
│   ┌──────────────┐     /rm/state_joint_state ┌──────────────┐               │
│   │ camera_node  │                           │ vr_teleop_   │               │
│   │  (RealSense) │                     ┌────►│    node      │               │
│   └──────┬───────┘                     │     └──────┬───────┘               │
│          │                             │            │                       │
│   /camera/*/color/image_raw            │     /rm/action_pose (8D)           │
│   /camera/*/depth/image_raw            │            │                       │
│          │                             │            ▼                       │
│          │                             │     ┌──────────────┐               │
│          │                             │     │realman_driver│               │
│          │                             └─────│    _node     │◄────┐         │
│          │                                   └──────┬───────┘     │         │
│          │                                          │             │         │
│          │                                   CANFD movep     UDP 状态        │
│          │                                          │             │         │
│          │                                          ▼             │         │
│          │                                   ┌──────────────┐     │         │
│          │                                   │  RM-75 机械臂 │─────┘         │
│          │                                   └──────────────┘               │
│          │                                                                  │
│          ▼                                                                  │
│   ┌──────────────┐                           ┌──────────────┐               │
│   │dataset_recor-│◄──────────────────────────│control_panel │               │
│   │  der_node    │     /recorder/status      │  (Web UI)    │               │
│   │ (数据录制)    │◄──────────────────────────│  :5000       │               │
│   └──────────────┘     /vr/teleop_status     └──────────────┘               │
│                                                                             │
└─────────────────────────────────────────────────────────────────────────────┘
```

---

## 节点功能说明

| 节点 | 功能 | 输入话题 | 输出话题 | Service |
|------|------|----------|----------|---------|
| `vr_input_node` | 读取 VR 控制器数据 | - | `/vr/right/pose`<br>`/vr/right/trigger`<br>`/vr/right/grip`<br>`/vr/connected` | - |
| `vr_teleop_node` | VR → 机械臂笛卡尔位姿映射 | `/vr/right/*`<br>`/rm/state_joint_state` | `/rm/action_pose` (8D)<br>`/vr/teleop_status` | `/vr/enable_control`<br>`/vr/disable_control` |
| `realman_driver_node` | 机械臂控制与状态读取 | `/rm/action_pose` | `/rm/state_joint_state` (14D)<br>`/driver/status` | `/driver/enable_follow`<br>`/driver/disable_follow` |
| `camera_node` | RealSense 相机采集 | - | `/camera/<name>/color/image_raw`<br>`/camera/<name>/depth/image_raw` | - |
| `dataset_recorder_node` | 同步录制 LeRobot 数据集 | 所有上述话题 | `/recorder/status` | `/recorder/start`<br>`/recorder/stop` |
| `control_panel` | Web UI 监控与控制 | 所有 status 话题 | - | 调用上述 Service |

---

## 通信拓扑图

### Topic 通信

```
VR 头显
    │
    ▼
vr_input_node ──► /vr/right/pose (PoseStamped)
              ──► /vr/right/trigger (Float32)
              ──► /vr/right/grip (Float32)
              ──► /vr/connected (Bool)
                        │
                        ▼
vr_teleop_node ──► /rm/action_pose (8D: x,y,z,qw,qx,qy,qz,gripper)
                        │
                        ▼
realman_driver_node ──► /rm/state_joint_state (14D)
                        │
                        ▼
camera_node ──► /camera/cam0/color/image_raw ──┐
            ──► /camera/cam0/depth/image_raw ──┼──► dataset_recorder_node
            ──► /camera/cam1/color/image_raw ──┤
            ──► /camera/cam1/depth/image_raw ──┘
```

### Service 通信

```
control_panel (Web UI)
    │
    ├── /vr/enable_control ─────────► vr_teleop_node   (启用 VR 控制)
    ├── /vr/disable_control ────────► vr_teleop_node   (禁用 VR 控制)
    ├── /driver/enable_follow ──────► realman_driver_node (启用跟随)
    ├── /driver/disable_follow ─────► realman_driver_node (禁用跟随)
    ├── /recorder/start ────────────► dataset_recorder_node (开始录制)
    └── /recorder/stop ─────────────► dataset_recorder_node (停止录制)
```

---

## 数据记录格式

本系统录制的数据集兼容 [LeRobot](https://github.com/huggingface/lerobot) 格式，同时参考 [RoboCOIN](https://github.com/FlagOpen/RoboCOIN) 数据集规范。

### 数据维度说明

| 字段 | 维度 | 说明 |
|------|------|------|
| `observation.state` | **14 维** | 机械臂真实状态（从 UDP 上报获取） |
| `action` | **8 维** | VR 控制指令（笛卡尔位姿 + 夹爪） |

### observation.state（14 维）

机械臂的实时状态，包含关节角度和末端位姿：

| 索引 | 名称 | 单位 | 说明 |
|------|------|------|------|
| 0-6 | `joint_1_rad` ~ `joint_7_rad` | rad | 7 个关节角度 |
| 7 | `gripper_open` | [0, 1] | 夹爪开合度（0=闭合，1=张开） |
| 8-10 | `eef_pos_x/y/z_m` | m | 末端位置（基座标系） |
| 11-13 | `eef_rot_euler_x/y/z_rad` | rad | 末端姿态（欧拉角 rx, ry, rz） |

### action（8 维）

VR 发送的笛卡尔控制指令：

| 索引 | 名称 | 单位 | 说明 |
|------|------|------|------|
| 0-2 | `x`, `y`, `z` | m | 目标末端位置 |
| 3-6 | `qw`, `qx`, `qy`, `qz` | - | 目标末端姿态（四元数） |
| 7 | `gripper` | [0, 1] | 夹爪目标开合度 |

### 数据集目录结构

```
data/local/realman_vr_xxx/
├── meta/
│   ├── info.json           # 数据集元信息（含 features 定义）
│   ├── episodes.jsonl      # 每个 episode 的帧数等信息
│   └── tasks.jsonl         # 任务描述
├── data/
│   └── chunk-000/
│       ├── episode_000000.parquet
│       └── ...
└── videos/
    └── chunk-000/
        ├── observation.images.cam0_rgb/
        │   └── episode_000000.mp4
        └── observation.images.cam0_depth_vis/
            └── episode_000000.mp4
```

---

## 环境准备

### 1. 系统要求

- Ubuntu 22.04 / 24.04
- ROS2 Humble / Jazzy
- Python 3.10+
- Conda (推荐使用 robocoin 环境)

### 2. 依赖安装

```bash
# ROS2 依赖
sudo apt install ros-${ROS_DISTRO}-cv-bridge ros-${ROS_DISTRO}-sensor-msgs

# Python 依赖
pip install numpy opencv-python flask pyserial scipy

# RealSense SDK (相机)
sudo apt install librealsense2-dev
pip install pyrealsense2

# ADB (VR 头显 USB 连接)
sudo apt install adb
```

### 3. 项目依赖

确保以下目录存在于仓库根目录：
- `Robotic_Arm/` - Realman 官方 Python API
- `r3kit/` - RealSense 驱动包

### 4. 设备权限

```bash
# RealSense 相机权限（需重新登录生效）
sudo cp /etc/udev/rules.d/99-realsense-libusb.rules /etc/udev/rules.d/
sudo udevadm control --reload-rules && sudo udevadm trigger

# ADB 权限
sudo usermod -aG plugdev $USER
```

### 5. 构建

```bash
# 激活 conda 环境
conda activate robocoin

# Source ROS2
source /opt/ros/${ROS_DISTRO}/setup.bash

# 构建
cd /home/ubuntu/Desktop/Workspace/vr_teleop/spacemouse_control_arm/ros2_realman_ws
colcon build --symlink-install

# Source 工作区
source install/setup.bash
```

---

## 配置说明

配置文件：`src/realman_teleop/config/teleop_params.yaml`

### VR 输入节点参数

| 参数 | 说明 | 默认值 |
|------|------|--------|
| `data_source` | VR 数据源类型 | `adb` |
| `adb_device_id` | ADB 设备 ID（空=自动检测） | `""` |
| `udp_binary_port` | WiFi 模式端口 | `12345` |
| `udp_json_port` | JSON 兼容模式端口 | `6666` |
| `publish_hz` | 发布频率 | `100.0` |
| `timeout` | 超时判定 | `1.0` |

### VR 数据源类型

| 数据源 | 描述 | 使用场景 |
|--------|------|----------|
| `adb` | USB 连接（推荐） | 默认推荐，稳定可靠 |
| `udp_binary` | WiFi 二进制数据 | 无 USB 线场景 |
| `udp_json` | JSON 兼容模式 | 兼容 hmi-user-dev |

### VR 遥操作节点参数

| 参数 | 说明 | 默认值 |
|------|------|--------|
| `control_hz` | 控制频率 | `100.0` |
| `trigger_threshold` | trigger 激活阈值 | `0.85` |
| `alpha_pos` | 位置滤波系数（越小越平滑） | `0.8` |
| `alpha_rot` | 旋转滤波系数 | `0.8` |
| `rotation_preset` | 旋转映射方案 | `perm_xyz_pnn` |

### 机械臂驱动参数

| 参数 | 说明 | 默认值 |
|------|------|--------|
| `dry_run` | 仿真模式（不连接真机） | `true` |
| `ip` | 机械臂 IP 地址 | `192.168.1.18` |
| `local_ip` | 本机 IP（UDP 状态接收） | `192.168.1.100` |
| `rm_plus_baud` | RM+ 协议波特率（夹爪） | `115200` |

### 数据录制参数

| 参数 | 说明 | 默认值 |
|------|------|--------|
| `camera_names` | 录制的相机列表 | `[cam0, cam1]` |
| `rotate_deg` | 相机图像旋转角度 | `0` |
| `repo_id` | 数据集 ID（每次录制需更换） | `local/xxx` |
| `record_hz` | 录制帧率 | `30.0` |

### 运行模式

```yaml
# 仿真模式（调试用，不连接真机）
dry_run: true

# 真机模式
dry_run: false
```

---

## 使用方法

### 一键启动

```bash
# 1. Source 环境
conda activate robocoin
source /opt/ros/${ROS_DISTRO}/setup.bash
source /home/ubuntu/Desktop/Workspace/vr_teleop/spacemouse_control_arm/ros2_realman_ws/install/setup.bash

# 2. 启动所有节点（默认 USB/ADB 连接）
ros2 launch realman_teleop vr_teleop.launch.py

# 或使用 WiFi 模式
ros2 launch realman_teleop vr_teleop.launch.py vr_data_source:=udp_binary

# 3. 打开浏览器访问 Web UI
# http://localhost:5000
```

**VR 模式操作流程：**
1. 启动 launch
2. 连接 VR 头显（USB 或确保同一局域网）
3. 打开 Web UI (http://localhost:5000)
4. 点击「启用 VR 控制」按钮
5. **按住 trigger（食指扳机）≥ 0.85** 开始控制
6. 移动手柄控制机械臂末端位姿
7. 按 grip（侧面握把）切换夹爪开/关
8. 点击「开始」录制数据
9. 点击「停止」结束录制

---

### 分终端启动（开发调试）

> 每个终端都需要先执行环境设置

```bash
# 每个终端都执行
conda activate robocoin
source /opt/ros/${ROS_DISTRO}/setup.bash
source /home/ubuntu/Desktop/Workspace/vr_teleop/spacemouse_control_arm/ros2_realman_ws/install/setup.bash

# 配置文件路径（后续命令中使用）
CONFIG=/home/ubuntu/Desktop/Workspace/vr_teleop/spacemouse_control_arm/ros2_realman_ws/src/realman_teleop/config/teleop_params.yaml
```

#### 步骤 1：启动机械臂驱动（终端 1）

```bash
ros2 run realman_teleop realman_driver_node --ros-args --params-file $CONFIG
```

验证：
```bash
ros2 topic hz /rm/state_joint_state
# 应显示约 30 Hz
```

#### 步骤 2：启动 VR 输入节点（终端 2）

```bash
ros2 run realman_teleop vr_input_node --ros-args --params-file $CONFIG
```

验证：
```bash
ros2 topic hz /vr/right/pose
# VR 连接后应显示约 100 Hz
```

#### 步骤 3：启动 VR 遥操作节点（终端 3）

```bash
ros2 run realman_teleop vr_teleop_node --ros-args --params-file $CONFIG
```

验证：
```bash
ros2 topic hz /rm/action_pose
# 应显示约 100 Hz
```

#### 步骤 4：启动相机（终端 4）

```bash
# 启动 cam0（腕部相机）
ros2 run realman_teleop camera_node --ros-args -r __node:=camera_node_cam0 --params-file $CONFIG

# 如需启动 cam1（终端 5）
ros2 run realman_teleop camera_node --ros-args -r __node:=camera_node_cam1 --params-file $CONFIG
```

验证：
```bash
ros2 topic hz /camera/cam0/color/image_raw
# 应显示约 30 Hz
```

#### 步骤 5：启用 VR 控制

```bash
# 方式 1：通过 Service 调用
ros2 service call /vr/enable_control std_srvs/srv/Trigger

# 方式 2：启动 Web UI 后点击按钮
ros2 run realman_teleop control_panel --ros-args -p mode:=vr -p camera_names:="['cam0', 'cam1']"
# 然后在浏览器中点击「启用 VR 控制」
```

#### 步骤 6：启动数据录制（终端 6）

```bash
ros2 run realman_teleop dataset_recorder_node --ros-args --params-file $CONFIG
```

控制录制：
```bash
# 开始录制
ros2 service call /recorder/start std_srvs/srv/Trigger

# 停止录制
ros2 service call /recorder/stop std_srvs/srv/Trigger
```

---

## Web UI 使用

### 功能说明

| 区域 | 功能 |
|------|------|
| VR 状态 | 显示连接状态、数据源、trigger/grip 值 |
| 节点状态 | 显示各节点在线状态和话题频率 |
| 录制状态 | 显示当前 Episode、帧数、时长 |
| 机械臂控制 | 启用/禁用 VR 控制 |
| 相机预览 | 实时显示 RGB + Depth 图像 |
| 系统日志 | 显示所有节点的日志信息 |

### 按钮说明

| 按钮 | 功能 | 说明 |
|------|------|------|
| 启用 VR 控制 | 启动 VR 遥操作 | 自动启用机械臂跟随 |
| 禁用 VR 控制 | 停止 VR 遥操作 | 自动禁用机械臂跟随 |
| 开始 | 开始录制 Episode | - |
| 停止 | 停止当前 Episode | - |

### VR 状态显示

```
VR 控制器状态
├── 连接状态: 已连接 / 未连接
├── 数据源: USB/ADB | WiFi | JSON
├── Trigger: [========  ] 0.85
├── Grip:    [====      ] 0.40
└── 位置: (0.123, 0.456, 0.789)
```

---

## VR 控制映射

| VR 输入 | 机械臂动作 | 阈值/逻辑 |
|---------|-----------|----------|
| Trigger (食指扳机) | 启用位姿跟踪 | ≥ 0.85 激活 |
| Grip (侧面握把) | Toggle 夹爪开/关 | 边缘检测，按一次切换 |
| 手柄位置差分 | 末端执行器位置 | EMA 滤波，坐标变换 |
| 手柄旋转差分 | 末端执行器姿态 | 四元数滤波 |

### 控制逻辑说明

1. **trigger 释放时**: 系统持续同步机械臂当前位姿作为锚点
2. **trigger 按下时**: 计算手柄位姿差分，映射到机械臂末端
3. **grip 边缘检测**: 避免重复触发，按一次切换夹爪状态

---

## 常见问题

### 1. VR 无数据

- 检查 USB 连接：`adb devices` 是否显示设备
- 确认 VR 应用已启动并发送数据
- 尝试 WiFi 模式：`ros2 launch realman_teleop vr_teleop.launch.py vr_data_source:=udp_binary`

### 2. 机械臂不动

- 检查 `dry_run` 是否为 `false`
- 确认已调用 `/vr/enable_control` 服务
- 检查 trigger 值是否 ≥ 0.85
- 检查网络连接：`ping 192.168.1.18`

### 3. 夹爪不动

- 检查 `rm_plus_baud` 参数是否正确（通常 115200 或 460800）
- 检查夹爪是否正确连接到末端接口

### 4. 位姿跟踪不准确

- 检查 `local_ip` 是否匹配实际局域网 IP
- 尝试不同的 `rotation_preset` 方案
- 调整滤波系数 `alpha_pos` 和 `alpha_rot`

### 5. 数据集目录已存在

- 修改 `repo_id` 参数，或删除已有目录

### 6. 未收到同步消息

- 检查所有话题是否有数据：`ros2 topic list`
- 放宽同步参数：增大 `sync_max_dt` 和 `cam_sync_max_dt`

### 7. UDP 状态超时

- 检查 `local_ip` 是否正确（不要使用 VPN IP）
- 确认防火墙未阻止 UDP 8089 端口

---

## 参数检查命令

```bash
# 检查 dry_run 模式
ros2 param get /realman_driver_node dry_run

# 检查 VR 连接状态
ros2 topic echo /vr/connected --once

# 查看所有话题
ros2 topic list

# 查看话题频率
ros2 topic hz /rm/state_joint_state    # 机械臂状态
ros2 topic hz /vr/right/pose           # VR 输入
ros2 topic hz /rm/action_pose          # VR 控制指令
ros2 topic hz /camera/cam0/color/image_raw

# 检查状态话题维度（应为 14 维）
ros2 topic echo /rm/state_joint_state --once | grep -A 20 "position:"

# 检查动作话题维度（应为 8 维）
ros2 topic echo /rm/action_pose --once | grep -A 10 "position:"
```

---

## 文件结构

```
ros2_realman_ws/
├── src/realman_teleop/
│   ├── config/
│   │   └── teleop_params.yaml       # 配置文件
│   ├── launch/
│   │   └── vr_teleop.launch.py      # VR 模式启动脚本
│   ├── realman_teleop/
│   │   ├── vr_input_node.py         # VR 输入节点
│   │   ├── vr_teleop_node.py        # VR 遥操作节点
│   │   ├── realman_driver_node.py   # 机械臂驱动节点
│   │   ├── camera_node.py           # 相机采集节点
│   │   ├── dataset_recorder_node.py # 数据录制节点
│   │   ├── hardware.py              # 硬件驱动封装
│   │   ├── vr_utils.py              # VR 工具函数
│   │   └── utils.py                 # 通用工具函数
│   └── web_ui/
│       └── control_panel.py         # Web UI 控制面板
└── README.md
```

---

## 硬件要求

- **机械臂**: Realman RM-75-6FB (7-DOF + 夹爪)
- **网络**: 机械臂 192.168.1.18，本机 192.168.1.x
- **相机**: Intel RealSense D435/D405（2 台：腕部 + 底座）
- **VR 头显**: Meta Quest / Pico（USB 调试或 WiFi）

---

## 更新日志

### 2026-02-04: 深度统计量存储优化

**问题背景**：
- 深度图（`dtype="uint8"`）的统计量按逐像素存储，导致 `episodes_stats.jsonl` 文件过大（150 episodes → 4GB）
- 数据集加载时间过长（5-10 分钟）
- 存包时间过长（30-50 秒/episode）

**优化内容**：
- 深度图统计量从逐像素 `(H, W)` 聚合为 scalar，与 RGB 统计量格式一致
- 深度图统计量单独存储到 `meta/episodes_stats_depth.jsonl`，不再与 RGB 统计量混存

**修改文件**：
- `realman_teleop/dataset_recorder_node.py`：添加 `write_episode_stats` hook

**预期效果**：
| 指标 | 优化前 | 优化后 | 提升 |
|------|--------|--------|------|
| `episodes_stats.jsonl` 大小 | 4GB (150 ep) | ~6MB | ~700x |
| 数据集加载时间 | 5-10 分钟 | 几秒 | ~100x |
| 单 episode 存包时间 | 30-50s | 15-25s | ~2x |

**兼容性**：
- 新采集的数据集使用优化后的格式
- 旧数据集可使用 `slim_episodes_stats.py` 脚本进行精简

---

## License

MIT License
