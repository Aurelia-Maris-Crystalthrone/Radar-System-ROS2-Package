import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node

def generate_launch_description():
    # 包路径
    livox_share = get_package_share_directory('livox_ros_driver2')
    point_lio_share = get_package_share_directory('point_lio')
    radar_system = get_package_share_directory('radar_system')  

    # 1. Livox 驱动
    livox_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([livox_share, 'launch', 'livox_driver.launch.py'])
        )
    )

    # 2. Point-LIO
    point_lio_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([point_lio_share, 'launch', 'point_lio.launch.py'])
        ),
        launch_arguments={
            'namespace': '',
            'rviz': 'False',
            'point_lio_cfg_dir': PathJoinSubstitution([point_lio_share, 'config', 'mid360.yaml'])
        }.items()
    )

    # 3. 基地坐标发布节点
    base_point_node = Node(
        package='your_package_name',
        executable='base_point_publisher',
        name='base_point_publisher',
        parameters=[{
            'frame_id': 'map',
            'base_x': 1.0,
            'base_y': 2.0,
            'base_z': 3.0
        }]
    )

    # 4. 距离计算节点
    distance_node = Node(
        package='radar_system',
        executable='radar_distance',
        name='radar_distance',
        parameters=[{
            'radar_odom_topic': '/aft_mapped_to_init',
            'base_point_topic': '/radar/base_point',
            'target_frame': 'map'         
        }]
    )

    return LaunchDescription([
        livox_launch,
        point_lio_launch,
        base_point_node,
        distance_node,
    ])