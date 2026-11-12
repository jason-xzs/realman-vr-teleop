# RealMan 双臂 VR 遥操作与数据采集

从 `/home/robot2/realman_teleop_v2` 整理的当前双臂采集代码，包含 ROS 2 后端、Flask 控制面板、HTML 前端、ADB/UDP VR 输入、三路 RealSense、RealMan SDK 和 LeRobot 数据录制。原目录保持不变。

## 代码入口

| 功能 | 文件 |
| --- | --- |
| 一键启动/停止 | [`quick_start.sh`](quick_start.sh) |
| 双臂 launch | [`vr_dual_teleop.launch.py`](spacemouse_control_arm/ros2_realman_ws/src/realman_teleop/launch/vr_dual_teleop.launch.py) |
| 双臂、相机、保存目录配置 | [`teleop_dual_params.yaml`](spacemouse_control_arm/ros2_realman_ws/src/realman_teleop/config/teleop_dual_params.yaml) |
| Web 后端 | [`control_panel.py`](spacemouse_control_arm/ros2_realman_ws/src/realman_teleop/web_ui/control_panel.py) |
| 当前 Web 前端 | [`control_panel.py` 中的 `INDEX_HTML`](spacemouse_control_arm/ros2_realman_ws/src/realman_teleop/web_ui/control_panel.py)（HTML/CSS/JavaScript 内嵌于后端文件） |
| 历史独立模板 | [`index.html`](spacemouse_control_arm/ros2_realman_ws/src/realman_teleop/web_ui/templates/index.html)（当前 `/` 路由使用上面的内嵌前端） |
| 机器人接口、录制器、VR 输入 | [`realman_teleop/`](spacemouse_control_arm/ros2_realman_ws/src/realman_teleop/realman_teleop) |

## 本机采集教程

1. USB 插上 VR，在头显界面打开右下角的小黑盒应用（用户提供的启动方式；APK 的对应关系尚未核实，仓库不含 APK）。
2. 主机执行 `adb devices`。若显示 `unauthorized`，执行下方命令，拔插 USB，并在 VR 中允许 USB 调试/连接，再检查设备列表。

   ```bash
   adb kill-server
   adb start-server
   adb devices
   ```

   状态必须为 `device`。`no permissions` 表示主机 USB/udev 权限问题，需要配置 adb 的设备规则或用户组；它与头显的授权弹窗是两个问题。

3. **每次新采集先修改保存名称**：打开 `teleop_dual_params.yaml`，修改 `dataset_recorder_node.ros__parameters.repo_id`，例如 `local/realman_vr_new_task_001`，并核对 `dataset_root`。本仓库默认 `dataset_root: ./data`，以 ROS 工作区为当前目录。配置文件虽以 `.yaml` 结尾，内容是带首部注释的 JSON，保留 JSON 语法。
4. 原机器现有环境的入口仍是：

   ```bash
   /home/robot2/realman_teleop_v2/quick_start.sh
   ```

   运行本仓库的副本时，先按下一节构建，再从仓库根目录启动：

   ```bash
   bash quick_start.sh
   ```

5. 浏览器打开 **http://localhost:5000**，启用 VR 控制；按住扳机才跟随，握把控制夹爪；在网页开始/停止录制。结束先停止录制，再 `bash quick_start.sh stop`。

现有双臂配置为 `dry_run=false`，会连接真机。启动脚本会清空两块专用机器人网卡的现有地址并重新配置路由；网卡默认 `enx6c1ff7eccd8e` 和 `enp89s0`，必须与实际设备对应。已手动配网时可设置 `SKIP_NETWORK_SETUP=1`。采集和推理不能同时占用机械臂控制连接。

## 环境与构建

本机已有 `jazzy_realman` 容器，挂载了整个 `/home/robot2`。使用这个容器时，先确认原容器依赖完整，再构建克隆的工作区：

```bash
docker start jazzy_realman
docker exec -it -w "$PWD/spacemouse_control_arm/ros2_realman_ws" jazzy_realman bash
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install
exit
bash quick_start.sh
```

新机器可使用提供的镜像配方（该镜像配方未在本次整理中实际构建）：

```bash
docker compose -f docker/compose.yaml up -d --build
docker exec -it realman_vr_teleop bash -lc \
  'cd /workspace/spacemouse_control_arm/ros2_realman_ws && source /opt/ros/jazzy/setup.bash && colcon build --symlink-install'
CONTAINER_NAME=realman_vr_teleop \
ROS_WS=/workspace/spacemouse_control_arm/ros2_realman_ws bash quick_start.sh
```

镜像使用 ROS 2 Jazzy、Python 3.12、LeRobot 0.3.3。ADB 与主机共享 `5037` 服务；先在主机完成 VR USB 授权。相机绑定和机械臂 IP 保留本机默认值，换设备时修改双臂配置。

## 数据格式

双臂 `observation.state` 与 `action` 均为 **28 维，左臂 14 + 右臂 14**。每臂为 7 关节（弧度）、夹爪（0–1）、末端位置（米）和欧拉角（弧度）。保存 episode 时 `action[t] = state[t+1]`，最后一帧保持当前状态。三路 RGB 为 `640x480@30`，腕部图像旋转 180°。

这份配置与历史单臂的 14D state / 8D VR action 有区别。详细接口和单位见 [`docs/realman_real_robot_experiment.md`](docs/realman_real_robot_experiment.md)；历史单臂说明见 [`docs/original_single_arm_README.md`](docs/original_single_arm_README.md)。数据、录像、Docker 镜像归档和缓存不上传。

## 来源与验证

来源文件及哈希见 [`docs/source_manifest.json`](docs/source_manifest.json)，第三方说明见 [`THIRD_PARTY_NOTICES.md`](THIRD_PARTY_NOTICES.md)。本次仅执行静态检查和离线测试，未启动真机、相机或采集流程。
