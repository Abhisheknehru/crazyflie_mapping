"""Phase 1 baseline: optical-flow-only pose -> RViz, with trajectory recording.

    ros2 launch crazyflie phase1_baseline.launch.py record_path:=baseline.csv

Brings up the UNCHANGED pose_estimator (raw flow-fused pose on /odom), the
of_odometry integrator (/of/pose + /of/path), and RViz. Drive the drone through
the maze, return to the start, Ctrl-C, then:

    python3 src/crazyflie/chectest/drift_analyze.py baseline.csv
"""
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import os


def generate_launch_description():
    rviz_config = os.path.join(
        get_package_share_directory('crazyflie'), 'rviz', 'crazyflie.rviz'
    )
    return LaunchDescription([
        DeclareLaunchArgument('uri', default_value='radio://0/80/2M/E7E7E7E7E7'),
        DeclareLaunchArgument('record_path', default_value=''),
        Node(
            package='crazyflie',
            executable='pose_estimator',
            output='screen',
            parameters=[{'uri': LaunchConfiguration('uri')}],
        ),
        Node(
            package='crazyflie',
            executable='of_odometry',
            output='screen',
            parameters=[{'record_path': LaunchConfiguration('record_path')}],
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            arguments=['-d', rviz_config],
            output='screen',
        ),
    ])
