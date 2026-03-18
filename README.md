# RM_LIDAR 雷达感知模块（英雄机器人吊射辅助）

## 项目概述
本项目为 **RobotMaster 超级对抗赛 2026** 定制开发的雷达感知模块，专门用于**英雄机器人吊射敌方基地**时的距离支持与辅助瞄准。模块基于 Livox MID-360 激光雷达，结合 ROS 框架实现实时战场环境感知、目标定位、距离解算，并将精确的距离信息发送至英雄机器人弹道解算节点，从而实现**高精度吊射**。

地图采用 `rmuc_2026.pgm`（对应超级对抗赛 2026 场地），通过雷达数据与地图匹配，为英雄机器人提供敌方基地的相对坐标和直线距离（误差 ≤ 2cm），辅助完成吊射诸元解算。

## 核心功能
| 功能项                | 详细说明                                                                 |
|-----------------------|--------------------------------------------------------------------------|
| 吊射距离解算          | 精准计算机器人炮口至敌方基地的直线距离，误差 ≤ 2cm，为弹道解算提供输入   |
| 敌方基地识别          | 基于先验地图与雷达点云，识别基地位置（可配合视觉辅助提高置信度）         |
| 实时定位与坐标系变换  | 将雷达坐标系下的目标转换至战场绝对坐标系（map），输出坐标 (x, y, θ)     |
| 动态目标过滤          | 滤除移动机器人、观众等干扰点云，仅保留静态基地/掩体等关键目标            |
| 异常数据容错          | 针对雷达遮挡、多路径干扰等做鲁棒处理，保证输出稳定性                     |
| 数据轻量化传输        | 通过 ROS 话题发布精简后的距离信息，降低通信负载                          |

## 技术栈
### 硬件适配
- **雷达传感器**：Livox MID-360 激光雷达（通过 Livox ROS Driver 2 驱动）
- **主控平台**：NVIDIA Jetson Orin / Xavier（运行 ROS 节点）
- **通信**：以太网（ROS 分布式通信）、CAN（与下位机弹道解算模块交互）

### 软件/算法
- **操作系统**：Ubuntu 22.04
- **ROS 版本**：ROS 2 Humble（推荐） / ROS 1 Noetic（可选）
- **核心依赖**：
  - [Livox ROS Driver 2](https://github.com/Livox-SDK/livox_ros_driver2)（雷达驱动）
  - [Point-LIO](https://github.com/hku-mars/Point-LIO)（激光惯性里程计，可选用于位姿估计）
  - [LOAM Interface](https://github.com/wh200720041/loam_interface)（激光里程计适配）
  - [Navigation2](https://navigation.ros.org/)（用于地图与路径规划，辅助定位）
  - BehaviorTree.CPP（行为树，用于任务调度）
- **核心算法**：
  - 卡尔曼滤波（点云降噪）
  - 聚类与目标提取（基于欧氏距离）
  - 坐标系变换（`tf2`）

## 快速开始
### 环境依赖
```bash
# 安装 ROS 2 Humble（参考官方文档）
# 安装 Livox ROS Driver 2
mkdir -p ~/rm_ws/src
cd ~/rm_ws/src
git clone https://github.com/Livox-SDK/livox_ros_driver2.git
# 编译
cd ~/rm_ws
colcon build --packages-select livox_ros_driver2

# 安装其他依赖
sudo apt install ros-humble-nav2-* ros-humble-behaviortree-cpp-v3
pip3 install transforms3d numpy matplotlib
```

### 部署步骤
1. **硬件连接**  
   - 将 Livox MID-360 通过以太网连接至 Jetson 主控（静态 IP 配置参考雷达手册）。
   - 确保雷达供电（12V / 24V，根据型号）稳定。

2. **配置雷达参数**  
   修改 `config/livox_config.json` 中的雷达 IP 和波特率，匹配实际设备。

3. **地图准备**  
   将比赛场地地图 `rmuc_2026.pgm` 及对应的 `rmuc_2026.yaml` 放入 `map/` 目录，并修改 `launch/navigation.launch.py` 中的地图路径。

4. **启动雷达感知模块**  
   ```bash
   cd ~/rm_ws
   source install/setup.bash
   ros2 launch rm_lidar bringup.launch.py
   ```
   该启动文件将依次启动：
   - Livox 雷达驱动
   - 点云预处理节点（`sensor_scan_generation`）
   - 自滤波节点（`self_filter_node`，用于去除机器人自身点云）
   - 目标提取与距离解算节点
   - TF 变换计算节点（`tf_diff_calculator`，用于计算雷达与机器人基座的相对位姿）
   - Navigation2 定位模块（可选，用于地图匹配）
   - RViz 可视化界面（默认配置 `rviz/nav2_default_view2.rviz`）

5. **查看距离输出**  
   距离信息通过 ROS 话题 `/target_distance` 发布，格式为 `std_msgs/Float64`。可订阅该话题并转发至下位机：
   ```bash
   ros2 topic echo /target_distance
   ```

## 代码结构说明
```
RM_LIDAR/
├── .vscode/                  # VS Code 配置（编译任务、调试）
├── bringup/                   # 启动相关
│   ├── behavior_tree/         # 行为树 XML 定义（如导航、定位、吊射触发）
│   ├── config/                # 参数配置文件
│   │   ├── nav2_params_terrain_adapt.yaml  # Navigation2 地形适配参数
│   │   └── nav2_params.yaml                 # Navigation2 默认参数
│   ├── launch/                # ROS 启动文件
│   │   ├── bringup.launch.py  # 总启动文件
│   │   └── navigation.launch.py # 仅启动导航相关节点
│   ├── map/                   # 场地地图
│   │   ├── rmuc_2026.pgm
│   │   └── rmuc_2026.yaml
|   ├── src/                       # 自定义源码
│   │   ├── self_filter_node_segment/  # 自滤波节点（去除机器人本体点云）
│   │   ├── self_filter_node.cpp       # 自滤波节点实现
│   │   └── tf_diff_calculator.cpp     # 计算雷达与敌方基地之间的 TF 差
│   ├── rviz/                  # RViz 可视化配置
│   │   ├── nav2_default_view.rviz
│   │   ├── nav2_default_view2.rviz
│   │   ├── test_map.rviz
│   │   ├── test.rviz
│   │   └── test2.rviz
│   ├── CMakeLists.txt
│   ├── LICENSE
│   └── package.xml
```

此外，项目中引用的外部仓库位于工作空间的 `src/` 下）：
```
src/
├── bringup                   # 自定义启动包（与 RM_LIDAR/bringup 配合）
├── livox_ros_driver2         # Livox 官方驱动
├── loam_interface            # LOAM 里程计接口
├── point_lio                 # Point-LIO 里程计
└── sensor_scan_generation    # 点云生成与预处理
```

## 吊射辅助专用说明
### 距离解算流程
1. **雷达点云获取**：通过 `livox_ros_driver2` 获取原始点云。
2. **自点云滤除**：`self_filter_node` 根据机器人几何模型滤除自身点云。
3. **目标提取**：对剩余点云进行聚类，结合地图先验提取敌方基地区域。
4. **距离计算**：计算基地点云质心与机器人炮口坐标的欧氏距离。
5. **输出**：发布 `/target_distance` 话题，同时可发布 `/target_marker` 用于 RViz 显示。

### 参数调优
- 修改 `config/nav2_params.yaml` 中的 `amcl` 参数可提高定位精度，从而间接提升距离解算精度。
- 在 `self_filter_node.cpp` 中调整 `robot_radius` 等参数可优化自身点云滤除效果。
- 目标提取聚类阈值可通过 `config/cluster_params.yaml` 设置（若已实现）。

## 赛事适配建议
1. **坐标系校准**：确保雷达与机器人基座的 TF 变换准确，可使用 `tf_diff_calculator` 动态计算偏差。
2. **地图更新**：比赛场地可能有微小变化，建议在赛前使用激光 SLAM 重新建图（可使用 `point_lio` 或 `loam_interface`）。
3. **通信接口**：若下位机需要特定格式的距离数据，可修改发布节点，适配自定义消息（如添加时间戳、置信度）。
4. **冗余设计**：同时运行视觉辅助识别基地，当雷达数据异常时切换至视觉数据，提高可靠性。

## 调试与常见问题
| 问题现象                  | 可能原因                          | 解决方案                                  |
|---------------------------|-----------------------------------|-------------------------------------------|
| 雷达无数据输出            | IP 配置错误 / 供电异常            | 检查雷达 IP（默认 192.168.1.50），ping 测试 |
| 距离输出跳变              | 基地点云聚类不稳定 / 自身滤除不彻底 | 增大聚类半径，检查 `self_filter_node` 参数 |
| TF 变换错误               | 雷达安装角度偏移                  | 重新标定，修改 `tf_diff_calculator` 中的偏移量 |
| Navigation2 定位漂移      | AMCL 参数不当 / 地图特征不足       | 调整粒子滤波器参数，增加雷达匹配权重      |

## 相关链接
- [QD 算法库](https://nomane-0.github.io/QD_Algorithm_Library/#/) —— 包含滤波、聚类、坐标变换等基础算法，可供参考或集成
- [PB2025 哨兵导航项目](https://gitee.com/SMBU-POLARBEAR/pb2025_sentry_nav) —— 类似的 Navigation2 配置与行为树示例，可借鉴其导航与定位模块

## 维护与贡献
- 维护者：[Aurelia-Maris-Crystalthrone]
- 联系方式：[3792458256@qq.com]
- 欢迎团队内部提交代码优化建议，提交前请确保通过测试脚本验证功能稳定性。

## 版权与声明
本项目为 RobotMaster 超级对抗赛 2026 参赛自研模块，仅供参赛学习使用，禁止商用。使用过程中需遵守机甲大师赛事规则，不得修改雷达硬件参数规避赛事限制。