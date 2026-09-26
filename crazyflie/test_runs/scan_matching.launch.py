"""Launch saved-map visualization and scan matching without a radio owner."""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    map_yaml = LaunchConfiguration('map_yaml')
    initial_x = LaunchConfiguration('initial_map_x')
    initial_y = LaunchConfiguration('initial_map_y')
    initial_yaw = LaunchConfiguration('initial_map_yaw_degrees')
    clearance = LaunchConfiguration('clearance')
    rviz_config = PathJoinSubstitution([
        FindPackageShare('crazyflie'), 'rviz', 'scan_matching.rviz'])
    return LaunchDescription([
        DeclareLaunchArgument(
            'map_yaml',
            default_value='maps/rviz_keyboard_20260922_171438_959977.yaml'),
        DeclareLaunchArgument('initial_map_x', default_value='0.0'),
        DeclareLaunchArgument('initial_map_y', default_value='0.0'),
        DeclareLaunchArgument(
            'initial_map_yaw_degrees', default_value='0.0'),
        DeclareLaunchArgument('clearance', default_value='0.10'),
        Node(
            package='crazyflie', executable='publish_saved_map',
            output='screen', parameters=[{'map_yaml': map_yaml}]),
        Node(
            package='crazyflie', executable='scan_match_localizer',
            output='screen', parameters=[{
                'map_yaml': map_yaml,
                'initial_map_x': ParameterValue(initial_x, value_type=float),
                'initial_map_y': ParameterValue(initial_y, value_type=float),
                'initial_map_yaw_degrees': ParameterValue(
                    initial_yaw, value_type=float),
            }]),
        Node(
            package='crazyflie', executable='a_star_planner',
            output='screen', parameters=[{
                'map_yaml': map_yaml,
                'clearance': ParameterValue(clearance, value_type=float),
            }]),
        Node(
            package='rviz2', executable='rviz2', output='screen',
            arguments=['-d', rviz_config]),
    ])
