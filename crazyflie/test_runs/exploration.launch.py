"""Connect one radio and feed the pose processor and exploration policy."""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, EmitEvent, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    bridge = Node(package='crazyflie', executable='exploration_radio_bridge',
                  output='screen', parameters=[{'uri': LaunchConfiguration('uri')}])
    pose = Node(package='crazyflie', executable='of_odometry', output='screen',
                parameters=[{'enable_constraint': ParameterValue(
                    LaunchConfiguration('enable_constraint'), value_type=bool),
                    'enable_mahalanobis': False,
                    'record_path': LaunchConfiguration('record_path')}])
    explorer = Node(package='crazyflie', executable='reactive_explorer', output='screen')
    nodes = [bridge, pose, explorer]
    return LaunchDescription([
        DeclareLaunchArgument('uri', default_value='radio://0/80/2M/E7E7E7E7E7'),
        DeclareLaunchArgument('enable_constraint', default_value='false'),
        DeclareLaunchArgument('record_path', default_value=''),
        *[RegisterEventHandler(OnProcessExit(target_action=node,
            on_exit=[EmitEvent(event=Shutdown(reason='Exploration component exited'))]))
          for node in nodes],
        *nodes,
    ])
