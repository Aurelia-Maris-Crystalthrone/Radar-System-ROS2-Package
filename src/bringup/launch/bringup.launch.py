import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, GroupAction,
                            IncludeLaunchDescription, TimerAction,
                            ExecuteProcess, SetEnvironmentVariable)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    # 设置环境变量，确保日志输出是缓冲的
    stdout_linebuf_envvar = SetEnvironmentVariable(
        'RCUTILS_LOGGING_BUFFERED_STREAM', '1'
    )

    # 获取 bringup 包的共享目录
    bringup_dir = get_package_share_directory('bringup')

    # 创建启动配置变量
    use_sim_time = LaunchConfiguration('use_sim_time')

    # 声明启动参数
    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation clock if true'
    )

    declare_map_yaml_cmd = DeclareLaunchArgument(
        "map",
        default_value=os.path.join(bringup_dir, "map", "rmuc_2026.yaml"),
        description="Full path to map file to load",
    )

    declare_params_file_cmd = DeclareLaunchArgument(
        "params_file",
        default_value=os.path.join(
            bringup_dir, "config", "nav2_params_terrain_analysis.yaml"
        ),
    )

    declare_slam_cmd = DeclareLaunchArgument(
        "slam", default_value="False", description="Whether run a SLAM"
    )

    declare_rviz_config_file_cmd = DeclareLaunchArgument(
        "rviz_config",
        default_value=os.path.join(bringup_dir, "rviz", "nav2_default_view.rviz"),
        description="Full path to the RViz config file to use",
    )

    # TF配置参数文件
    tf_config_file = LaunchConfiguration('tf_config_file')
    declare_tf_config_file_cmd = DeclareLaunchArgument(
        'tf_config_file',
        default_value=os.path.join(bringup_dir, 'config', 'tf_config.yaml'),
        description='Full path to TF configuration file'
    )
    rviz_config_file = LaunchConfiguration("rviz_config")

    # 指定动作组（包含所有需要并发启动的节点和包含启动）
    bringup_cmd_group = GroupAction([
        # ----- 新增：启动 map_server（生命周期节点）-----
        Node(
            package='nav2_map_server',
            executable='map_server',
            name='map_server',
            parameters=[{
                'yaml_filename': LaunchConfiguration('map'),
                'use_sim_time': use_sim_time
            }],
            output='screen',
        ),

        # 添加静态坐标发布
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='map_to_odom',
            arguments=['0', '0', '0', '0', '0', '0', 'map', 'odom']
        ),
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='base_footprint_to_livox_frame',
            arguments=['0', '-0.1682', '0.396', '0', '0', '0', 'base_footprint', 'livox_frame']
        ),
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='enermy_base_to_odom',
            arguments=['21.5', '0', '0', '0', '0', '0', 'odom', 'enermy_base']
        ),
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='aft_mapped_to_body',
            arguments=['0', '0', '0', '0', '0', '0', 'aft_mapped', 'body']
        ),
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='body_to_base_footprint',
            arguments=['0', '0', '0', '0', '0', '0', 'body', 'base_footprint']
        ),
        # 包含导航启动（navigation_launch.py）
        IncludeLaunchDescription(
            launch_description_source=PythonLaunchDescriptionSource([
                PathJoinSubstitution([
                    FindPackageShare('bringup'),
                    'launch',
                    'navigation_launch.py'
                ])
            ]),
            launch_arguments={
                'use_sim_time': use_sim_time,
                'map': LaunchConfiguration('map')
            }.items()
        ),

        # 启动 livox_ros_driver2
        IncludeLaunchDescription(
            launch_description_source=PythonLaunchDescriptionSource([
                PathJoinSubstitution([
                    FindPackageShare('livox_ros_driver2'),
                    'launch',
                    'msg_MID360_launch.py'
                ])
            ]),
            launch_arguments={'use_sim_time': use_sim_time}.items()
        ),

        # 启动 point_lio
        IncludeLaunchDescription(
            launch_description_source=PythonLaunchDescriptionSource([
                PathJoinSubstitution([
                    FindPackageShare('point_lio'),
                    'launch',
                    'point_lio.launch.py'
                ])
            ]),
            launch_arguments={'use_sim_time': use_sim_time}.items()
        ),
        IncludeLaunchDescription(
            launch_description_source=PythonLaunchDescriptionSource([
                PathJoinSubstitution([
                    FindPackageShare('loam_interface'),
                    'launch',
                    'loam_interface_launch.py'
                ])
            ]),
            launch_arguments={'use_sim_time': use_sim_time}.items()
        ),
        IncludeLaunchDescription(
            launch_description_source=PythonLaunchDescriptionSource([
                PathJoinSubstitution([
                    FindPackageShare('sensor_scan_generation'),
                    'launch',
                    'sensor_scan_generation.launch.py'
                ])
            ]),
            launch_arguments={'use_sim_time': use_sim_time}.items()
        ),
        # 5. 启动self_filter_node
        Node(
                package='bringup',  
                executable='self_filter_node', 
                name='self_filter_node',
                output='screen',
                parameters=[{'use_sim_time': use_sim_time}
            ]),
        # 启动 rviz2（使用声明的配置文件）
        Node(
            package="rviz2",
            executable="rviz2",
            arguments=["-d", rviz_config_file],
            output="screen",
        )
    ])

    # 创建 lifecycle_bringup 进程，用于激活 map_server
    lifecycle_bringup_process = ExecuteProcess(
        cmd=['ros2', 'run', 'nav2_util', 'lifecycle_bringup', 'map_server'],
        name='lifecycle_bringup_map_server',
        output='screen',
    )

    # 延迟 2 秒执行 lifecycle_bringup，确保 map_server 已完全启动
    delayed_lifecycle_bringup = TimerAction(
        period=2.0,
        actions=[lifecycle_bringup_process]
    )

    # 创建启动描述
    ld = LaunchDescription()

    # 添加环境变量
    ld.add_action(stdout_linebuf_envvar)

    # 添加参数声明
    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(declare_map_yaml_cmd)
    ld.add_action(declare_params_file_cmd)
    ld.add_action(declare_rviz_config_file_cmd)
    ld.add_action(declare_tf_config_file_cmd)

    # 添加启动所有节点的动作组
    ld.add_action(bringup_cmd_group)

    # 添加延迟的 lifecycle_bringup
    ld.add_action(delayed_lifecycle_bringup)

    return ld