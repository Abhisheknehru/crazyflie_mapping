from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription, LaunchContext
from launch.actions import DeclareLaunchArgument, OpaqueFunction, TimerAction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
import os


def create_nodes(context: LaunchContext):
    package_share = get_package_share_directory('crazyflie')
    rviz_config = os.path.join(package_share, 'rviz', 'mapping.rviz')
    resolution = float(LaunchConfiguration('resolution').perform(context))
    arena_width = float(LaunchConfiguration('arena_width').perform(context))
    arena_height = float(LaunchConfiguration('arena_height').perform(context))
    width = round(arena_width / resolution)
    height = round(arena_height / resolution)
    origin_x = -arena_width / 2.0
    origin_y = -arena_height / 2.0
    startup_delay = float(LaunchConfiguration('startup_delay').perform(context))

    mapper = Node(
        package='crazyflie',
        executable='esp_mapper',
        output='screen',
        parameters=[{
            'pose_topic': LaunchConfiguration('pose_topic'),
            'pose_type': LaunchConfiguration('pose_type'),
            'map_topic': 'esp/map',
            'resolution': resolution,
            'width': width,
            'height': height,
            'origin_x': origin_x,
            'origin_y': origin_y,
            'fixed_yaw': ParameterValue(
                LaunchConfiguration('fixed_yaw'), value_type=float,
            ),
            'use_fixed_yaw': ParameterValue(
                LaunchConfiguration('use_fixed_yaw'), value_type=bool,
            ),
            'range_filter_window': ParameterValue(
                LaunchConfiguration('range_filter_window'), value_type=int,
            ),
        }],
    )
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', rviz_config],
        output='screen',
    )

    return [
        mapper,
        TimerAction(period=startup_delay, actions=[rviz]),
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('arena_width', default_value='8.0'),
        DeclareLaunchArgument('arena_height', default_value='8.0'),
        DeclareLaunchArgument('resolution', default_value='0.04'),
        DeclareLaunchArgument('pose_topic', default_value='/of/pose'),
        DeclareLaunchArgument('pose_type', default_value='pose_stamped'),
        DeclareLaunchArgument('fixed_yaw', default_value='0.0'),
        DeclareLaunchArgument('use_fixed_yaw', default_value='false'),
        DeclareLaunchArgument('range_filter_window', default_value='7'),
        DeclareLaunchArgument('startup_delay', default_value='2.0'),
        OpaqueFunction(function=create_nodes),
    ])
