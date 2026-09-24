"""Launch saved-map visualization and scan matching without a radio owner."""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    map_yaml = LaunchConfiguration('map_yaml')
    rviz_config = PathJoinSubstitution([
        FindPackageShare('crazyflie'), 'rviz', 'scan_matching.rviz'])
    return LaunchDescription([
        DeclareLaunchArgument(
            'map_yaml',
            default_value='maps/rviz_keyboard_20260922_171438_959977.yaml'),
        Node(
            package='crazyflie', executable='publish_saved_map',
            output='screen', parameters=[{'map_yaml': map_yaml}]),
        Node(
            package='crazyflie', executable='scan_match_localizer',
            output='screen', parameters=[{'map_yaml': map_yaml}]),
        Node(
            package='rviz2', executable='rviz2', output='screen',
            arguments=['-d', rviz_config]),
    ])
