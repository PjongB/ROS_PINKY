"""Launch Pinky Nav2 with this project's map and Nav2 parameters."""

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import AnyLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description() -> LaunchDescription:
    """Include upstream bringup without modifying its source files."""
    project_share = FindPackageShare('pinky_project_1')
    navigation_share = FindPackageShare('pinky_navigation')

    return LaunchDescription([
        IncludeLaunchDescription(
            AnyLaunchDescriptionSource(
                PathJoinSubstitution([
                    navigation_share,
                    'launch',
                    'gz_bringup_launch.xml',
                ])
            ),
            launch_arguments={
                'use_sim_time': 'true',
                'use_composition': 'False',
                'map': PathJoinSubstitution([
                    project_share,
                    'map',
                    'tutorial_map.yaml',
                ]),
                'params_file': PathJoinSubstitution([
                    project_share,
                    'config',
                    'nav2_params.yaml',
                ]),
            }.items(),
        ),
    ])
