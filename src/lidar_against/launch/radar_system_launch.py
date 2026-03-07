from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    # 声明可配置的参数
    base_x_arg = DeclareLaunchArgument(
        'base_x', default_value='0.0',
        description='基地坐标 X (前向为正)'
    )
    base_y_arg = DeclareLaunchArgument(
        'base_y', default_value='0.0',
        description='基地坐标 Y (左向为正)'
    )
    base_z_arg = DeclareLaunchArgument(
        'base_z', default_value='0.0',
        description='基地坐标 Z (上向为正)'
    )
    frame_id_arg = DeclareLaunchArgument(
        'frame_id', default_value='map',
        description='所有点使用的坐标系'
    )
    publish_rate_arg = DeclareLaunchArgument(
        'publish_rate', default_value='10.0',
        description='发布频率 (Hz)'
    )
    radius_arg = DeclareLaunchArgument(
        'radius', default_value='10.0',
        description='雷达圆周运动半径 (米)'
    )
    angular_velocity_arg = DeclareLaunchArgument(
        'angular_velocity', default_value='0.2',
        description='雷达角速度 (rad/s)'
    )
    center_x_arg = DeclareLaunchArgument(
        'center_x', default_value='0.0',
        description='圆周运动中心 X 坐标'
    )
    center_y_arg = DeclareLaunchArgument(
        'center_y', default_value='0.0',
        description='圆周运动中心 Y 坐标'
    )
    center_z_arg = DeclareLaunchArgument(
        'center_z', default_value='2.0',
        description='雷达固定高度 Z'
    )

    # 节点1: 雷达位置发布 (话题 /radar/position)
    radar_position_node = Node(
        package='radar_system',         
        executable='radar_position_publisher',
        name='radar_position_publisher',
        parameters=[{
            'radius': LaunchConfiguration('radius'),
            'angular_velocity': LaunchConfiguration('angular_velocity'),
            'center_x': LaunchConfiguration('center_x'),
            'center_y': LaunchConfiguration('center_y'),
            'center_z': LaunchConfiguration('center_z'),
            'frame_id': LaunchConfiguration('frame_id'),
            'publish_rate': LaunchConfiguration('publish_rate'),
        }],
        output='screen'
    )

    # 节点2: 基地位置发布 (话题 /radar/base_point)
    base_point_node = Node(
        package='radar_system',
        executable='base_point_publisher',
        name='base_point_publisher',
        parameters=[{
            'base_x': LaunchConfiguration('base_x'),
            'base_y': LaunchConfiguration('base_y'),
            'base_z': LaunchConfiguration('base_z'),
            'frame_id': LaunchConfiguration('frame_id'),
            'publish_rate': LaunchConfiguration('publish_rate'),
        }],
        output='screen'
    )

    # 节点3: 距离计算 (订阅 /radar/position 和 /radar/base_point，发布 /radar/distance)
    distance_node = Node(
        package='radar_system',
        executable='radar_distance',
        name='radar_distance',
        output='screen'
    )

    # 节点4: 距离监视器 (订阅 /radar/distance 并打印)
    monitor_node = Node(
        package='radar_system',
        executable='distance_monitor',   
        name='distance_monitor',
        output='screen'
    )

    return LaunchDescription([
        base_x_arg,
        base_y_arg,
        base_z_arg,
        frame_id_arg,
        publish_rate_arg,
        radius_arg,
        angular_velocity_arg,
        center_x_arg,
        center_y_arg,
        center_z_arg,
        radar_position_node,
        base_point_node,
        distance_node,
        monitor_node,
    ])