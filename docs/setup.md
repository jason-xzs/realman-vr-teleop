# 安装与排障

## 复用已有 jazzy_realman 容器

在已安装采集环境的工作站上，可以跳过镜像构建。以下命令均从仓库根目录执行；仓库需位于容器已挂载的 `/home/robot2` 下。

```bash
bash setup.sh --existing-container jazzy_realman
CONTAINER_NAME=jazzy_realman \
ROS_WS="$PWD/spacemouse_control_arm/ros2_realman_ws" bash quick_start.sh
```

停止时指定相同的容器：

```bash
CONTAINER_NAME=jazzy_realman bash quick_start.sh stop
```

## 机器人网络

左右臂使用相同网段，通过两条独立的 `/32` 路由区分。

| | 默认网口 | 主机地址 | 机械臂地址 | UDP 状态端口 |
| --- | --- | --- | --- | --- |
| 左臂 | `enx6c1ff7eccd8e` | `192.168.1.100` | `192.168.1.18` | `8090` |
| 右臂 | `enp89s0` | `192.168.1.101` | `192.168.1.19` | `8089` |

用 `ip link` 查看主机网口名称。换网卡时在启动命令中指定：

```bash
LEFT_IFACE=enxYOUR_LEFT_ADAPTER RIGHT_IFACE=enpYOUR_RIGHT_ADAPTER \
  bash quick_start.sh
```

单独配置网络，不启动采集：

```bash
bash quick_start.sh network
ping -c 2 192.168.1.18
ping -c 2 192.168.1.19
```

网络脚本只配置指定的机器人网口和目标路由，不清空已有网卡地址。更改机器人 IP 或主机 IP 时，同时更新双臂配置中的 `ip`、`local_ip`，以及网络脚本的环境变量 `LEFT_ARM_IP`、`RIGHT_ARM_IP`、`LEFT_HOST_IP`、`RIGHT_HOST_IP`。

已经手动配置好网络时：

```bash
SKIP_NETWORK_SETUP=1 bash quick_start.sh
```

## 数据写入外部硬盘

Compose 将仓库映射为容器的 `/workspace`。默认数据会直接落在主机仓库的 ROS 工作区下。

需要使用仓库外的目录时，在 `docker/compose.yaml` 的 `volumes` 中增加挂载，例如 `/mnt/robot-data:/datasets`，重建容器后将双臂配置中的 `dataset_root` 设置为 `/datasets`。这里填写容器内路径。

## VR 不在线

| 现象 | 处理 |
| --- | --- |
| `adb devices` 没有设备 | 换数据线或 USB 口，确认头显已开机且 USB 调试已打开 |
| `unauthorized` | 在头显允许调试；必要时重启 ADB 并拔插 USB |
| `no permissions` | 安装 `android-sdk-platform-tools-common`，确认 USB udev 规则和用户权限，重新插拔设备 |
| 主机看到设备，容器看不到 | 先在主机运行 `adb start-server`，再执行 `docker exec realman_vr_teleop adb devices`；Compose 使用主机 ADB 的 `5037` 端口 |
| ADB 为 `device`，网页 VR 仍离线 | 启动头显里的配套采集应用，确认 `adb logcat` 中持续出现 `VRDeviceData` |

## 相机不出图

先停止其他相机程序或推理客户端，确认三路相机通过 USB 3 连接。换相机后更新配置中的序列号。可以在主机查询设备，不打开视频流：

```bash
python3 -c 'import pyrealsense2 as rs; print([d.get_info(rs.camera_info.serial_number) for d in rs.context().query_devices()])'
```

主机未安装 `pyrealsense2` 时，在采集容器中运行同一命令。

## 网页打不开或节点启动失败

检查 `bash quick_start.sh` 的终端日志；网页默认使用 `5000` 端口。模块导入错误可重新执行 `bash setup.sh` 复核依赖。已有容器模式下使用 `bash setup.sh --existing-container jazzy_realman`。

网页服务应在工作站或可信局域网中访问，不提供用户登录认证。
