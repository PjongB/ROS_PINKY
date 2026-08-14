"""Launch the patrol manager and docking controller."""

from launch import LaunchDescription
from launch.actions import Shutdown
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution


def generate_launch_description() -> LaunchDescription:
    params_file = PathJoinSubstitution([
        FindPackageShare('pinky_project_1'),
        'config',
        'project_params.yaml',
    ])

    return LaunchDescription([
        Node(
            package='pinky_project_1',
            executable='patrol_manager',
            name='patrol_manager',
            output='screen',
            parameters=[params_file],
            on_exit=Shutdown(
                reason='Patrol mission finished; stopping pj_project.'
            ),
        ),
        Node(
            package='pinky_project_1',
            executable='docking_controller',
            name='docking_controller',
            output='screen',
            parameters=[params_file],
        ),
    ])
