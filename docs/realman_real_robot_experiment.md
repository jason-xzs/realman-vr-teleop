# RealMan 真机实验说明

面向这台主机上已经跑通过的双臂 RM 采集和推理代码。后续自己写控制、回放或新任务时，按这里的接口、单位和启动顺序做，避免和现有数据对不上。

代码位置：

| 用途 | 路径 |
| --- | --- |
| 当前采集入口 | `/home/robot2/realman_teleop_v2/quick_start.sh` |
| 双臂配置 | `realman_teleop_v2/spacemouse_control_arm/ros2_realman_ws/src/realman_teleop/config/teleop_dual_params.yaml` |
| 驱动封装 | `.../realman_teleop/hardware.py` |
| 状态发布 | `.../realman_teleop/realman_driver_node.py` |
| 录制 | `.../realman_teleop/dataset_recorder_node.py` |
| 官方 Python SDK | `/home/robot2/realman_teleop_v2/Robotic_Arm/rm_robot_interface.py` |
| 推理侧硬件封装 | `/home/robot2/deltawam/src/realman_deployment/hardware.py` |

`spacemouse_control_arm/teleop_collect_realman*.py` 是更早的外骨骼/SpaceMouse 单脚本采集，当前双臂 VR 数据不走这条路径。仓库里的 `ros2_realman_ws/README.md` 仍写单臂和旧路径，以 `quick_start.sh` 和 `teleop_dual_params.yaml` 为准。

## 1. 真机前

同一时刻只能有一个进程连机械臂。ROS launch 开着时，不要再跑自己的 SDK 脚本，反过来也一样。两路 TCP 都是 8080，后连上的会把控制权抢走。

急停在手边。第一次只读状态，确认 IP、关节数、夹爪读数后再发运动。运动先用 `rm_movej` 阻塞、速度百分比 10 以内，目标用当前关节附近的小偏移，确认方向后再上 CANFD 透传。

当前双臂配置里 `dry_run` 已是 `false`，launch 会直接连真机。调试节点逻辑时先改回 `true`。

## 2. 网络和地址

`quick_start.sh` 会给两块网卡配主机地址，并按目标臂加 /32 路由：

| | 机械臂 IP | 主机地址 | 控制端口 | UDP 状态端口 |
| --- | --- | --- | --- | --- |
| 左臂 | `192.168.1.18` | `192.168.1.100`（`enx6c1ff7eccd8e`） | 8080 | 8090 |
| 右臂 | `192.168.1.19` | `192.168.1.101`（`enp89s0`） | 8080 | 8089 |

网卡名写死在脚本里。换机器或换 USB 网卡后先用 `ip link` 核对，再改 `setup_network()`。

相机（`teleop_dual_params.yaml`）：

| 名字 | 序列号 | 旋转 |
| --- | --- | --- |
| `left_wrist` | `323622272614` | 180° |
| `right_wrist` | `230422272673` | 180° |
| `high` | `420122071413` | 0° |

三路都是 30 Hz 彩色，当前不录深度。腕部 180° 是训练数据的朝向，推理客户端上传前也会转腕部图。

## 3. 启动采集

容器名是 `jazzy_realman`，ROS 2 Jazzy 在容器里。

```bash
cd /home/robot2/realman_teleop_v2
./quick_start.sh          # 配网并在容器里 ros2 launch vr_dual_teleop.launch.py
./quick_start.sh stop     # 向容器里的 launch 发 SIGINT
```

launch 会拉起：左右 `realman_driver_node`、一个 `vr_input_node`（同时发左右手柄）、左右 `vr_teleop_node`、三路 `camera_node`、`dataset_recorder_node`、Web UI `control_panel`（`http://localhost:5000`）。

采集前改 `dataset_recorder_node.repo_id`，否则会往旧数据集里继续写。当前值是 `local/realman_teleop_vr_bowls_0828_20`。数据根目录：

`/home/robot2/realman_teleop_v2/spacemouse_control_arm/ros2_realman_ws/data`

操作顺序：浏览器打开 Web UI，启用 VR 控制，按住扳机（阈值 0.85）才跟随，握把切夹爪，再点开始/停止录制。停进程用 `./quick_start.sh stop`。容器里的 PID 和宿主机不一样，不要拿宿主机 PID 去 `docker exec kill`。

## 4. SDK：连接、运动、读状态

SDK 类是 `RoboticArm`。采集侧用 `RM_TRIPLE_MODE_E`；推理侧左臂用 `RM_DUAL_MODE_E`，右臂用默认构造。多线程模式下 `block=1` 会等到到位或规划失败。

连接：

```python
from Robotic_Arm.rm_ctypes_wrap import rm_thread_mode_e
from Robotic_Arm.rm_robot_interface import RoboticArm

arm = RoboticArm(rm_thread_mode_e.RM_TRIPLE_MODE_E)
handle = arm.rm_create_robot_arm("192.168.1.18", 8080)
# handle.id == -1 表示连接失败
arm.rm_set_arm_run_mode(1)   # SDK 定义：0 仿真，1 真实
# 用完
arm.rm_delete_robot_arm()
```

`hardware.py` 里把 `rm_set_arm_run_mode(1)` 注释成“实时模式”，以 SDK 注释为准：1 是真实模式。

返回码在各运动接口上一致：`0` 成功，`1` 控制器拒绝（参数或状态错误），`-1` 发送失败，`-2` 接收失败或超时，`-3` 解析失败。阻塞运动还可能返回 `-4`（到位设备不是关节）、`-5`（单线程超时）。

### 该用哪个运动接口

| 接口 | 何时用 | 单位 |
| --- | --- | --- |
| `rm_movej(joint, v, r, connect, block)` | 回 home、点到点。`v` 是速度百分比 1–100，`block=1` 等到位 | 关节，度 |
| `rm_movel` / `rm_movej_p` | 走直线或关节空间到某个末端位姿 | 位置米，欧拉角弧度 |
| `rm_movej_canfd(joint, follow, expand, trajectory_mode, radio)` | 闭环跟踪、推理 30 Hz 下发。不经规划，发完即返回 | 关节，度 |
| `rm_movep_canfd(pose, follow, trajectory_mode, radio)` | VR 遥操作。控制器内部 IK，关节变化不大才直接下发 | 6 维欧拉角，或 7 维四元数；位置米，欧拉角弧度 |
| `rm_set_gripper_position(position, block, timeout)` | 夹爪。`position` 范围 1–1000，不是 0–1 | 无量纲 |

CANFD 参数（当前双臂配置）：

- `follow=False`：低跟随。高跟随要求透传周期不超过 10 ms，周期不稳会抖。
- `trajectory_mode=0`：完全透传。`1` 曲线拟合（`radio` 0–100），`2` 滤波（`radio` 0–999）。
- `rm_movej_canfd` 没有速度百分比。速度由相邻目标的差和发送周期决定。目标跳变会直接打到关节上。

现有回 home 就是阻塞 `rm_movej`。左臂 `home_velocity=40`，右臂 `home_velocity=20`，单位都是百分比。home 关节在 `teleop_dual_params.yaml` 的 `home_joint_deg`，单位度。默认 home 完成后驱动会把夹爪开到 1000；需要保持夹持时传 `open_gripper=False`。

急停：采集封装调用 `rm_set_arm_emergency_stop(True)`。推理封装在异常和 Ctrl-C 时调用 `rm_set_arm_slow_stop()`，再 `rm_delete_robot_arm()`。自己写循环时，`finally` 里至少做一次停止再断开。

### 读状态的两条路

推理客户端用 SDK 查询，适合自己写的短实验：

```python
code, state = arm.rm_get_current_arm_state()
# state["joint"]: 7 维，度
# state["pose"]:  [x, y, z, rx, ry, rz]，米 / 弧度
code, grip = arm.rm_get_gripper_state()
# grip["actpos"]: 1–1000
```

采集驱动不用这条读关节。它让机械臂 UDP 主动上报，本机 `UDPStateReceiver` 收 JSON，消息类型必须是 `realtime_arm_joint_state`。原始单位和换算：

| 字段 | 原始单位 | 换算后 |
| --- | --- | --- |
| `joint_status.joint_position` | 0.001° | 除以 1000 得度，再 `np.radians` 写入数据集 |
| `waypoint.position` | 微米 | 除以 1e6 得米 |
| `waypoint.euler` | 0.001 rad | 除以 1000 得弧度 |
| `rm_plus_state.pos[0]` | 0–1000 | 除以 1000 得 `gripper_open` |

`rm_get_gripper_state()` 读的是标准 Modbus 夹爪，RM+ 生态夹爪的真实开合在 UDP 的 `rm_plus_state.pos[0]`。采集状态用的是后者。UDP 读到 0 且本周期发过夹爪命令时，驱动会退回最后一次命令，避免把没上报的夹爪记成闭合。

### 夹爪协议

`rm_set_gripper_position` 合法范围是 1–1000，驱动里会把 0 夹到 1。数据集里的 `gripper_open` 是 0–1，下发前乘 1000。

RM+ 生态协议和工具端 RS485 夹爪互斥。采集连接时用 JSON `{"command":"set_rm_plus_mode","mode":115200}` 打开 RM+；退出前必须再发 `mode: 0`，否则末端生态可能挂掉。`RealmanDriver.close()` 已经做了这件事，自己直接用 SDK 时要同样处理。驱动发夹爪前还会 `rm_set_tool_rs485_mode(2, 115200)`，避免某侧还停在 RTU/生态模式把命令吞掉。

## 5. 数据集里记了什么

每臂 14 维，双臂按 left、right 拼成 28 维。`observation.state` 和 `action` 同维、同名。

单臂顺序（双臂时名前加 `left_` / `right_`）：

```
joint_1_rad … joint_7_rad
gripper_open          # 0 闭，1 开
eef_pos_x_m, eef_pos_y_m, eef_pos_z_m
eef_rot_euler_x_rad, eef_rot_euler_y_rad, eef_rot_euler_z_rad
```

末端是基坐标系下的绝对反馈，不是相对第一帧，也没有做差分。关节写入前已从度转成弧度。

`action` 不是 VR 指令。录制时先用当前 state 占位，保存 episode 时做 `action[t] = state[t+1]`，最后一帧保持 `action = state`。所以这批数据是绝对状态跟踪：看到 `state[t]`，预测下一时刻的 28 维状态。

同步：`record_hz=30`，状态与图像 `sync_max_dt=0.05` s，相机之间 `cam_sync_max_dt=0.03` s。图像是 RGB，腕部已按配置旋转。

## 6. 和推理客户端的差别

`deltawam` 读到的 28 维 state 与采集相同：左 14 + 右 14，关节弧度，夹爪 0–1，末端米/弧度。

模型动作是 16 维，且左右顺序和 state 相反：

```
[0:7]   右臂 7 关节，弧度
[7]     右夹爪
[8:15]  左臂 7 关节，弧度
[15]    左夹爪
```

下发前 `np.rad2deg`，再 `rm_movej_canfd(..., follow=False)`。默认单步关节变化上限 0.5 rad，超限直接拒绝。夹爪默认走 binary hysteresis：数值 ≤300 关到位置 1，≥700 开到 1000，中间保持；连续 3 帧才切换。连续位置模式才把 0–1000 直接下发。

因此：

- 用采集数据训练出来的 28 维 `action[t]=state[t+1]`，不能原样塞进现在的 16 维推理客户端。
- 16 维动作里没有末端位姿，只有关节和夹爪。
- 推理先无 `EXECUTE` 跑 dry-run，确认相机、关节顺序和日志后再 `EXECUTE=1`。

## 7. 自己写真机实验时

1. 停掉 `jazzy_realman` 里的 launch，确认 8080 没有别的连接。
2. 只连一条臂，`rm_get_current_arm_state()` 打印关节（度）和末端（米、弧度）。自由度应为 7。
3. 回 home 用配置里的 `home_joint_deg`，`rm_movej(..., v=10, r=0, connect=0, block=1)`。速度从 10 开始，不要一上来用配置里的 20/40。
4. 跟踪循环用 `rm_movej_canfd`，低跟随，周期稳定在 20–30 ms 量级。每拍自己做关节步长限制，CANFD 不会替你限速。
5. 位姿控制用 `rm_movep_canfd` 时传米和弧度（6 维）或米加四元数（7 维）。不要把 UDP 原始整数直接送回去。
6. 夹爪用 1–1000。退出时关 RM+（若打开过）并 `rm_delete_robot_arm()`。
7. 要和现有数据集对齐，记录 14 维时用弧度、夹爪 0–1、末端米和弧度，双臂左在前右在后。要对齐推理客户端，动作改成右在前的 16 维。

SDK 源码注释在 `Robotic_Arm/rm_robot_interface.py`：连接约 7115 行，`rm_movej` 约 1460 行，`rm_movej_canfd` 约 1736 行，`rm_movep_canfd` 约 1769 行，夹爪约 3356 行。本机封装的换算和退出顺序以 `hardware.py` 为准。
