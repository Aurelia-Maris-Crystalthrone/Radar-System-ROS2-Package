# RM_LIDAR 雷达感知与导航集成模块 (ROS 2)

[![ROS 2](https://img.shields.io/badge/ROS_2-Humble-blue)](https://docs.ros.org/en/humble/index.html)
[![License](https://img.shields.io/badge/License-MIT-green)](LICENSE)

## 1. 项目概述

本项目是一个专为 **RobotMaster 超级对抗赛 2026** 英雄机器人设计的集成化 ROS 2 工作空间。它集成了基于 **Livox MID-360** 激光雷达的实时环境感知、激光惯性里程计、自定义感知数据处理以及完整的 Navigation 2 导航框架。

其核心目标是为下位机（如弹道解算单元）提供经过处理的**实时距离、位置数据**，并支持机器人在比赛场地内进行自主导航与避障。

## 2. 系统架构与数据流

系统由多个 ROS 2 包协同工作，形成一个完整的数据处理链路：

| 模块 | 功能 | 关键节点 |
| :--- | :--- | :--- |
| **传感器驱动** | 驱动 Livox MID-360 雷达，发布原始点云。 | `livox_ros_driver2` |
| **里程计 (Odometry)** | 通过 Point-LIO 算法融合 IMU 和激光雷达数据，提供高频、低漂移的位姿估计。 | `point_lio` |
| **地形分析** | 将三维点云转化为二维代价地图 (Costmap)，用于导航。 | `terrain_analysis` |
| **感知处理** | 自定义节点，负责去除机器人自身的点云和整理数据。 | `self_filter_node`, `radar_to_serial_node` |
| **导航框架** | 提供全局/局部路径规划、控制、行为树等完整的自主导航功能。 | `planner_server`, `controller_server`, `bt_navigator` |
| **坐标变换** | 管理机器人各部件之间的 TF 变换关系。 | `tf_diff_calculator` |

**核心数据流:**

```mermaid
graph TD
    A[Livox MID-360] -->|/livox/lidar| B(point_lio)
    A -->|/livox/lidar| E(sensor_scan_generation)
    B -->|/Odometry| C(loam_interface)
    C -->|/cloud_registered| D(terrain_analysis)
    D -->|/terrain_map| F(Nav2 Costmap)
    C -->|/aft_mapped_to_init| G(tf_diff_calculator)
    A -->|/livox/lidar| H(self_filter_node)
    G -->|/radar/target_position| I(radar_to_serial_node)
    H -->|/pointclouds_self_filter| I
    I -->|Serial Data| J[下位机 (MCU)]
```

### 项目文件结构

```
Radar-System-ROS2-Package/
├── bringup/                          # 核心启动与集成包
│   ├── CMakeLists.txt
│   ├── package.xml
│   ├── launch/
│   │   ├── bringup.launch.py        # 系统总启动文件
│   │   ├── navigation_launch.py     # Nav2 导航子启动
│   │   └── pointlio_launch.py       # Point-LIO 子启动
│   ├── config/
│   │   ├── nav2_params_terrain_analysis.yaml  # 导航参数配置
│   │   └── mapper_params_online.yaml          # 定位建图参数
│   ├── src/
│   │   ├── self_filter_node.cpp               # 自身点云滤除节点
│   │   ├── radar_to_serial_node.cpp           # 串口数据转发节点
│   │   └── sensor_scan_generation_node.cpp    # 扫描生成节点
│   └── include/
│       └── bringup/
├── terrain_analysis/                 # 地形分析包
│   ├── CMakeLists.txt
│   ├── package.xml
│   └── src/
│       └── terrain_analysis.cpp
├── loam_interface/                   # 里程计接口适配包
│   ├── CMakeLists.txt
│   ├── package.xml
│   └── src/
│       └── loam_interface_node.cpp
├── point_lio/                        # Point-LIO 激光惯性里程计
│   ├── CMakeLists.txt
│   ├── package.xml
│   ├── include/point_lio/
│   └── src/
├── livox_ros_driver2/                # Livox 雷达 ROS 2 驱动
│   ├── CMakeLists.txt
│   ├── package.xml
│   ├── launch_ROS2/
│   ├── config/
│   │   └── MID360_config.json        # 雷达网络配置
│   └── src/
├── sensor_scan_generation/           # 扫描数据生成包
│   ├── CMakeLists.txt
│   └── package.xml
├── tf_diff_calculator/               # TF 坐标差值计算包
│   ├── CMakeLists.txt
│   └── package.xml
├── navigation2/                      # Nav2 官方包（作为子模块或拷贝）
│   └── (nav2_* 系列包)
├── README.md
└── LICENSE
```

> **说明**：实际开发中主要关注 `bringup` 包内的启动与配置文件，以及 `livox_ros_driver2/config/MID360_config.json` 中的雷达网络设置。

## 3. 节点详解 (bringup 包)

此部分详细说明了 `bringup` 包中实现的自定义功能节点。

- **`self_filter_node`**: 通过一个固定的立方体范围 (`CropBox`) 滤除属于机器人本体的点云。这对于防止机器人在导航时将自身部件误识别为障碍物至关重要。
- **`tf_diff_calculator`**: 一个简单的 TF 监听节点。它以 10Hz 的频率查询 `odom` 坐标系与 `enermy_base` 坐标系之间的变换，并将其作为 `PointStamped` 消息发布到 `/radar/target_position` 话题。
- **`radar_to_serial_node`**: 数据聚合与通信节点。它订阅 `/radar/distance`、`/radar/position` 和 `/radar/base_point` 三个话题，将数据整合后，通过配置的串口（默认 `/dev/ttyUSB0`, 115200 波特率）以 20Hz 的频率发送给下位机。

## 4. 环境与硬件要求

### 4.1 软件依赖

- **操作系统**: Ubuntu 22.04
- **ROS 2 发行版**: Humble
- **核心依赖**:
  - `ros-humble-nav2-*`
  - `ros-humble-behaviortree-cpp-v3`
  - `python3-transforms3d`, `python3-numpy`, `python3-matplotlib`
  - `pcl_conversions` (点云处理)
  - `serial` (串口通信)

### 4.2 硬件要求

- **激光雷达**: Livox MID-360
- **主控平台**: NVIDIA Jetson Orin / Xavier (推荐) 或性能相当的 x86 工控机
- **通信接口**: 以太网 (连接雷达) 与 UART/CAN (通过串口与下位机通信)

## 5. 快速开始

### 5.1 创建工作空间并克隆代码

```bash
mkdir -p ~/rm_ws/src
cd ~/rm_ws/src
git clone https://github.com/Aurelia-Maris-Crystalthrone/Radar-System-ROS2-Package.git .
```

### 5.2 安装依赖

```bash
# 安装 ROS 2 依赖
cd ~/rm_ws
rosdep install --from-paths src --ignore-src -r -y

# 安装 Python 依赖
pip3 install transforms3d numpy matplotlib pyserial
```

### 5.3 编译工作空间

```bash
cd ~/rm_ws
colcon build --symlink-install
source install/setup.bash
```

## 6. 配置指南

启动前，请根据实际情况修改以下配置文件：

1.  **雷达 IP 配置**: 修改 `src/livox_ros_driver2/config/MID360_config.json`，确保 `lidar_configs` 下的 IP 地址与你的雷达一致。
2.  **TF 变换配置**: 在 `src/bringup/launch/bringup.launch.py` 中，修改 `base_footprint_to_livox_frame` 等静态 TF 发布者的 `arguments` 参数，以匹配雷达在机器人上的实际安装位置。
3.  **串口配置**: 启动时可通过参数指定串口设备和波特率，或修改启动文件中的默认值 (`/dev/ttyUSB0`, `115200`)。
4.  **导航参数**: 根据机器人尺寸和性能，调整 `src/bringup/config/nav2_params_terrain_analysis.yaml` 中的机器人半径 (`robot_radius`)、速度 (`max_velocity`) 等参数。
5.  **自滤波范围**: 在 `src/bringup/src/self_filter_node_segmentation.cpp` 中，调整 `CropBox` 的 `min_pt` 和 `max_pt` 向量值，以确保能精确滤除机器人自身的点云。

## 7. 运行系统

使用主启动文件 `bringup.launch.py` 即可启动所有模块：

```bash
# 启动全部功能
ros2 launch bringup bringup.launch.py

# 带参数启动示例：指定地图和RViz配置文件
ros2 launch bringup bringup.launch.py map:=/path/to/your/map.yaml rviz_config:=/path/to/your/rviz.rviz
```

启动文件将按顺序启动雷达驱动、定位、感知处理和导航框架。

## 8. 关键 ROS 话题

| 话题名 | 消息类型 | 描述 |
| :--- | :--- | :--- |
| `/livox/lidar` | `sensor_msgs/PointCloud2` | 原始点云数据。 |
| `/cloud_registered` | `sensor_msgs/PointCloud2` | 经里程计校正后的点云。 |
| `/radar/target_position` | `geometry_msgs/PointStamped` | 目标点在 `enermy_base` 坐标系下的位置。 |
| `/radar/distance` | `std_msgs/Float64` | 需要外部计算并发布，供 `radar_to_serial_node` 转发。 |
| `/terrain_map` | `sensor_msgs/PointCloud2` | 用于导航的二维地形图。 |
| `/cmd_vel` | `geometry_msgs/Twist` | Navigation 2 输出的速度指令。 |

## 9. 故障排除

- **雷达无数据**: 检查雷达供电、网线连接，确认 `MID360_config.json` 中的 IP 配置正确，并尝试 ping 雷达 IP。
- **TF 变换错误**: 使用 `tf2_tools view_frames` 命令生成 TF 树，检查是否存在断连。确认 `bringup.launch.py` 中的静态 TF 发布者参数无误。
- **串口发送失败**: 确认用户有串口读写权限 (`sudo usermod -a -G dialout $USER`)，并检查启动参数 `serial_port` 是否正确。
- **导航定位漂移**: 检查里程计 (`/odom`) 是否准确，并确保 `amcl` 参数文件中的地图路径正确。可尝试调大 `controller_server` 中的 `transform_tolerance` 参数。

## 10. 贡献与许可

- **维护者**: [Aurelia-Maris-Crystalthrone]
- **联系方式**: [3792458256@qq.com]
- **许可证**: 本项目代码遵循 MIT 协议。请注意，部分依赖项（如 Navigation 2）可能有其自身的许可条款。