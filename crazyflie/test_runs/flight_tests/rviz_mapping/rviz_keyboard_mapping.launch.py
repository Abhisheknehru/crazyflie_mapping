"""Keyboard flight window with yaw-aware point cloud displayed in RViz."""
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
import os


def generate_launch_description():
    rviz_config = os.path.join(
        get_package_share_directory('crazyflie'),
        'rviz', 'rviz_mapping', 'rviz_keyboard_mapping.rviz')
    return LaunchDescription([
        DeclareLaunchArgument(
            'uri', default_value='radio://0/80/2M/E7E7E7E7E7'),
        DeclareLaunchArgument('size', default_value='2.5'),
        DeclareLaunchArgument('resolution', default_value='0.03'),
        DeclareLaunchArgument('map_save_prefix', default_value='maps/rviz_keyboard'),
        Node(
            package='crazyflie',
            executable='rviz_keyboard_mapping',
            output='screen',
            emulate_tty=True,
            parameters=[{'uri': LaunchConfiguration('uri')}],
        ),
        Node(
            package='crazyflie',
            executable='multiranger_mapper',
            output='screen',
            parameters=[{
                'pose_topic': '/crazyflie/pose',
                'size': ParameterValue(
                    LaunchConfiguration('size'), value_type=float),
                'resolution': ParameterValue(
                    LaunchConfiguration('resolution'), value_type=float),
                # A wall can be about 1.25 m away from the arena centre.
                # Keep a little hysteresis below the script's 2 m sensor limit.
                'trust_near': 1.8,
                'trust_far': 2.0,
                'map_save_prefix': LaunchConfiguration('map_save_prefix'),
            }],
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            arguments=['-d', rviz_config],
            output='screen',
        ),
    ])
