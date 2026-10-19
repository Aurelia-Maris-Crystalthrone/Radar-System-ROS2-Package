# 第三方组件与版权声明

本仓库自研部分仅 `src/bringup` 下的三个 ROS 2 节点、`include/bringup/radar_data_converter.hpp`
与相应的 launch / config。其余均为第三方代码，版权归各自作者所有。

## 随本仓库 vendored（直接搬运 / 改写）

| 路径 | 来源 | 许可证 |
| :--- | :--- | :--- |
| `src/livox_ros_driver2/` | Livox SDK 官方 ROS 2 驱动，`Livox-SDK/livox_ros_driver2` | MIT |
| `src/livox_ros_driver2/3rdparty/rapidjson/` | Tencent RapidJSON | MIT（含 BSD 组件，见其 `license.txt`） |
| `src/bringup/lib/serial/` | William Woodall / John Harrison，`wjwwood/serial` | MIT |
| `src/loam_interface/` | 源自 Lihan Chen 等人（CMU 地面移动机器人栈）的 `loam_interface`，本仓库仅调整了坐标系与话题参数 | Apache-2.0 |
| `src/sensor_scan_generation/` | 同上，源自同一套 `sensor_scan_generation` | Apache-2.0 |
| `src/bringup/config/nav2_params*.yaml` | 结构与参数取法参考 `SMBU-PolarBear-Robotics-Team/pb2025_sentry_nav` | Apache-2.0 / GPLv3（依上游仓库许可） |

已在对应目录保留其原始 LICENSE 文件；`loam_interface` 与 `sensor_scan_generation`
保留了源文件头的 Apache-2.0 copyright 声明，未做删除或改写。

## 未随本仓库发布（见 `dependency.repos`）

| 功能包 | 作用 | 许可证 |
| :--- | :--- | :--- |
| `point_lio`（hku-mars/Point-LIO） | 激光惯性里程计 | GPLv2（商业用途需联系作者） |
| `pb_nav2_plugins`（SMBU-PolarBear-Robotics-Team） | 提供 `pb_nav2_costmap_2d::IntensityVoxelLayer` | 见上游仓库 |
| `terrain_analysis` | 产出 `/terrain_map`、`/terrain_map_ext` | 见上游仓库 |

> 注意：Point-LIO 采用 GPLv2。若本工程用于**非研究性的商业部署**，请按 GPLv2 要求
> 履行相应义务，或替换为许可更宽松的里程计方案（如 FAST-LIO2 系列同样需注意其许可条款）。

## 关于命名

仓库名为 `Radar-System-ROS2-Package`，但传感器为 **Livox MID-360 激光雷达（LiDAR）**，
与毫米波 / 相控阵雷达无关。命名沿用队内工程习惯，特此说明避免误解。
