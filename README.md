# RealMan VR Teleop

使用 VR 手柄遥操作 RealMan RM75 双臂，通过网页查看三路相机、控制机械臂并录制 LeRobot 数据集。

- 双手柄分别控制左右臂，支持扳机跟随和夹爪控制
- 三路 RealSense RGB 同步采集，30 FPS
- Web 控制面板：归位、跟随、录制、保存与丢弃 episode
- ROS 2 Jazzy + Docker，包含 RealMan Python/C SDK

策略部署见 [realman-policy-inference](https://github.com/jason-xzs/realman-policy-inference)。

## 硬件连接

默认配置对应 RM75 双臂工作站。使用同一套设备时，保留以下配置即可。

| 设备 | 连接 / 配置 |
| --- | --- |
| 左臂 | 网口 `enx6c1ff7eccd8e`；机械臂 `192.168.1.18:8080`；主机 `192.168.1.100` |
| 右臂 | 网口 `enp89s0`；机械臂 `192.168.1.19:8080`；主机 `192.168.1.101` |
| 左腕相机 | USB 3；序列号 `323622272614` |
| 右腕相机 | USB 3；序列号 `230422272673` |
| 顶部相机 | USB 3；序列号 `420122071413` |
| VR 头显 | USB 数据线；开启 USB 调试并运行配套的 VR 采集应用 |

两条机械臂网线分别接入对应网口，相机接 USB 3，头显用支持数据传输的 USB 线连接主机。采集应用需要输出 `VRDeviceData` 日志；仅有 ADB 连接还不能产生手柄数据。

## 安装

主机需要 Ubuntu x86_64、Docker Engine、Docker Compose v2，以及当前用户的 Docker 访问权限。ROS 和 Python 采集依赖由容器安装。

```bash
sudo apt install adb android-sdk-platform-tools-common
git clone https://github.com/jason-xzs/realman-vr-teleop.git
cd realman-vr-teleop
bash setup.sh
```

`setup.sh` 构建容器和 ROS 工作区，并检查 SDK、相机、录制器和网页模块能否导入。安装完成后容器名为 `realman_vr_teleop`，此时不会启动机器人节点。

已有 `jazzy_realman` 环境的工作站可以直接复用容器，见 [复用已有环境](docs/setup.md#复用已有-jazzy_realman-容器)。

## 开始采集

### 1. 连接 VR

戴上头显，打开配套采集应用。此工作站已安装的应用从头显右下角的小黑盒入口打开。

```bash
adb devices
```

列表中应显示 `device`。如果是 `unauthorized`，执行以下命令，重新插拔 USB，并在头显中允许 USB 调试：

```bash
adb kill-server
adb start-server
adb devices
```

可以用以下命令确认应用正在发送手柄数据：

```bash
adb logcat | grep -m 1 VRDeviceData
```

### 2. 设置数据集名称

编辑 [teleop_dual_params.yaml](spacemouse_control_arm/ros2_realman_ws/src/realman_teleop/config/teleop_dual_params.yaml) 中的 `dataset_recorder_node.ros__parameters`：

```json
"repo_id": "local/pick_place_001",
"dataset_root": "./data",
"task": "Place the objects into the container."
```

新任务或新一轮采集使用新的 `repo_id`；继续同一个数据集则保留原名称。配置修改后重新启动采集。文件采用 JSON 语法，保留引号和逗号。

### 3. 启动

```bash
bash quick_start.sh
```

脚本配置机器人专用网口，并启动双臂、VR、三路相机、录制器和网页。浏览器打开 **[http://localhost:5000](http://localhost:5000)**。

### 4. 录制 episode

1. 确认网页中的左右臂、三路相机和 VR 均在线。
2. 在左右臂面板中点击 `Home` 归位，再启用 `VR` 控制。
3. 按住对应手柄的扳机移动机械臂，松开扳机停止跟随；握把控制夹爪。
4. 点击录制区的 **开始**，完成操作后点击 **停止**。
5. 在弹窗中选择 **保存** 或 **丢弃**；等待保存完成后继续下一条。

操作前确认 Home 位姿周围无障碍、急停可触达。采集与策略推理应分开运行，避免两个进程同时占用机械臂和相机。

### 5. 结束

先保存或丢弃当前 episode，再停止节点：

```bash
bash quick_start.sh stop
```

关闭容器可使用 `docker compose -f docker/compose.yaml stop`。下次先运行 `docker compose -f docker/compose.yaml start`，再运行 `bash quick_start.sh`。

## 数据保存

默认写入仓库内的 ROS 工作区：

```text
spacemouse_control_arm/ros2_realman_ws/data/local/pick_place_001/
├── meta/      # 数据集、任务、episode 和统计信息
├── data/      # Parquet 状态与动作
└── videos/    # 三路相机视频
```

双臂状态和动作均为 28D，按左臂、右臂排列。每臂包括 7 个关节角、夹爪开度、末端位置和欧拉角；关节与欧拉角为弧度，位置为米，夹爪范围为 0–1。动作使用下一帧状态 `action[t] = state[t+1]`，最后一帧保持当前状态。

## 更换设备或调整配置

| 项目 | 修改位置 |
| --- | --- |
| 机械臂 IP、Home 位姿、速度、相机序列号 | [双臂配置](spacemouse_control_arm/ros2_realman_ws/src/realman_teleop/config/teleop_dual_params.yaml) |
| 主机网口、主机 IP 和机器人路由 | [网络配置说明](docs/setup.md#机器人网络) |
| 外部数据盘、已有容器、VR 授权问题 | [安装与排障](docs/setup.md) |

SDK 接口与数据格式详见 [RealMan 接口说明](docs/realman_real_robot_experiment.md)。第三方组件许可见 [THIRD_PARTY_NOTICES.md](THIRD_PARTY_NOTICES.md)。
