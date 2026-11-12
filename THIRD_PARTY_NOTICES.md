# 来源与第三方组件

- ROS/Web 采集代码来自本机 `realman_teleop_v2/spacemouse_control_arm`；原有 MIT 许可保留于 `LICENSE`。
- `Robotic_Arm/` 来自本机 RealMan 官方 Python/C SDK，包含原生动态库。上游：https://github.com/RealManRobot/RM_API2 。SDK 的使用与分发遵循上游条款，根目录的 MIT 许可不用于替代其条款。
- `r3kit/` 是本机配套的设备封装及本地修改，原文件版权声明予以保留。
- LeRobot 为外部安装依赖，采用 Apache-2.0：https://github.com/huggingface/lerobot 。

本次整理保留源文件，另增加启动路径兼容、容器配方和文档。来源哈希反映整理前文件；导出哈希及修改清单记录于 `docs/source_manifest.json`。
