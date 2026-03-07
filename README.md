Radar System ROS2 Package

本项目是一个ROS2功能包，用于模拟雷达系统，发布雷达自身位置、基地位置，并计算两者之间的距离。它包含四个节点，围绕话题 /radar/position、/radar/base_point 和 /radar/distance 进行通信。所有坐标均遵循 x向前为正，y向左为正，z向上为正 的右手坐标系。
功能概述

    雷达位置发布节点 (radar_position_publisher)：以可配置的圆周运动模拟雷达实时位置，发布到 /radar/position。

    基地位置发布节点 (base_point_publisher)：发布固定的基地坐标到 /radar/base_point。

    距离计算节点 (radar_distance)：订阅上述两个话题，计算欧几里得距离，并发布到 /radar/distance。

    距离监视节点 (distance_monitor)：订阅 /radar/distance 并打印距离值，用于演示或调试。

通过提供的 launch 文件，可以一键启动所有节点，便于快速测试和集成。
依赖

    ROS2 发行版（如 Humble、Iron 或 Rolling）

    标准消息包：geometry_msgs、std_msgs

    C++ 编译环境

编译

    将本包放入你的 ROS2 工作空间（例如 ~/ros2_ws/src）。

    安装依赖（如有缺失）：
    bash

    cd ~/ros2_ws
    rosdep install --from-paths src --ignore-src -r -y

    编译：
    bash

    colcon build --packages-select <your_package_name>
    source install/setup.bash

用法
使用 launch 文件启动所有节点

ros2 launch <your_package_name> radar_system_launch.py

你可以通过命令行参数覆盖默认配置，例如：
bash

ros2 launch <your_package_name> radar_system_launch.py base_x:=5.0 radius:=20.0 angular_velocity:=0.5

单独运行节点

你也可以分别启动各个节点，以便调试或集成到其他系统中：
bash

# 终端1：雷达位置
ros2 run <your_package_name> radar_position_publisher

# 终端2：基地位置
ros2 run <your_package_name> base_point_publisher

# 终端3：距离计算
ros2 run <your_package_name> radar_distance

# 终端4：距离监视（可选）
ros2 run <your_package_name> distance_monitor

查看话题数据
bash

# 雷达位置
ros2 topic echo /radar/position

# 基地位置
ros2 topic echo /radar/base_point

# 距离
ros2 topic echo /radar/distance

节点说明
1. radar_position_publisher

    发布话题：/radar/position (geometry_msgs/msg/PointStamped)

    功能：模拟雷达做圆周运动，周期性发布当前位置。

    可配置参数（可通过 launch 文件或动态参数设置）：

        radius：圆周半径（米），默认 10.0

        angular_velocity：角速度（rad/s），默认 0.2

        center_x, center_y, center_z：圆周运动中心坐标，默认 (0.0, 0.0, 2.0)

        frame_id：坐标系名称，默认 "map"

        publish_rate：发布频率（Hz），默认 10.0

2. base_point_publisher

    发布话题：/radar/base_point (geometry_msgs/msg/PointStamped)

    功能：发布固定的基地坐标。

    可配置参数：

        base_x, base_y, base_z：基地坐标，默认 (0.0, 0.0, 0.0)

        frame_id：坐标系名称，默认 "map"

        publish_rate：发布频率（Hz），默认 10.0

3. radar_distance

    订阅话题：

        /radar/position (geometry_msgs/msg/PointStamped)

        /radar/base_point (geometry_msgs/msg/PointStamped)

    发布话题：/radar/distance (std_msgs/msg/Float64)

    功能：计算雷达与基地的欧几里得距离，并发布。

    说明：节点使用最新收到的两个点进行计算，不进行时间同步。如果两个点的 frame_id 不同，会发出警告。

4. distance_monitor

    订阅话题：/radar/distance (std_msgs/msg/Float64)

    功能：简单的监视节点，将收到的距离值打印到终端。

    用途：可作为调试工具，也可替换为其他处理节点。

话题列表
话题名	消息类型	发布节点	描述
/radar/position	geometry_msgs/msg/PointStamped	radar_position_publisher	雷达实时位置
/radar/base_point	geometry_msgs/msg/PointStamped	base_point_publisher	基地固定位置
/radar/distance	std_msgs/msg/Float64	radar_distance	雷达与基地的距离
参数配置

所有节点参数均可在 launch 文件中通过命令行覆盖。例如：
bash

ros2 launch <your_package_name> radar_system_launch.py \
    base_x:=1.0 base_y:=2.0 base_z:=3.0 \
    radius:=15.0 angular_velocity:=0.3 \
    publish_rate:=5.0

完整参数列表见各节点说明。
坐标系约定

为确保距离计算正确，所有点消息均使用同一坐标系（默认 map），且坐标轴方向为：

    x：前向为正

    y：左向为正

    z：上向为正

扩展与定制

    如需接入真实雷达数据，可修改 radar_position_publisher 的 timer_callback 函数，替换为从传感器或定位模块读取实际坐标。

    若需要多雷达或多基地，可复制节点并修改话题名。

    距离计算节点可以扩展为支持多个点对，或加入时间同步机制（如 message_filters）。

许可证

本项目采用 MIT 许可证。详情请见 LICENSE 文件。

如有问题或建议，欢迎提交 Issue 或 Pull Request。
判断其价值

# Radar-System-ROS2-Package
