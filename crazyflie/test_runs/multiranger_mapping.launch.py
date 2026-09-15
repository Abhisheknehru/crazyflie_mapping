"""Run the mapper and preconfigured RViz alongside the existing radio startup."""
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    config = os.path.join(get_package_share_directory('crazyflie'),
                          'rviz', 'multiranger.rviz')
    return LaunchDescription([
        DeclareLaunchArgument('size', default_value='10.0'),
        DeclareLaunchArgument('resolution', default_value='0.02'),
        Node(package='crazyflie', executable='multiranger_mapper', output='screen',
             parameters=[{name: ParameterValue(LaunchConfiguration(name), value_type=float)
                          for name in ('size', 'resolution')}]),
        Node(package='rviz2', executable='rviz2', arguments=['-d', config], output='screen'),
    ])
