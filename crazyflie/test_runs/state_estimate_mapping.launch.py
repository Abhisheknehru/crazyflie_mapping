"""Radio stateEstimate + Multi-ranger mapping."""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, EmitEvent, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    bridge = Node(
        package='crazyflie', executable='state_estimate_bridge',
        output='screen', parameters=[{
            'uri': LaunchConfiguration('uri'),
            'settle_timeout': ParameterValue(
                LaunchConfiguration('settle_timeout'), value_type=float),
        }])
    mapper = Node(
        package='crazyflie', executable='multiranger_mapper', output='screen',
        parameters=[{
            'pose_topic': '/crazyflie/pose',
            'map_save_prefix': LaunchConfiguration('map_save_prefix'),
            'size': ParameterValue(LaunchConfiguration('size'), value_type=float),
            'resolution': ParameterValue(
                LaunchConfiguration('resolution'), value_type=float),
        }])
    return LaunchDescription([
        DeclareLaunchArgument(
            'uri', default_value='radio://0/80/2M/E7E7E7E7E7'),
        DeclareLaunchArgument('settle_timeout', default_value='60.0'),
        DeclareLaunchArgument('size', default_value='10.0'),
        DeclareLaunchArgument('resolution', default_value='0.03'),
        DeclareLaunchArgument('map_save_prefix', default_value='maps/maze'),
        RegisterEventHandler(OnProcessExit(
            target_action=bridge,
            on_exit=[EmitEvent(event=Shutdown(reason='Radio bridge exited'))])),
        bridge, mapper,
    ])
