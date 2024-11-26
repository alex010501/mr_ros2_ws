import os
import sys

import launch
import launch_ros.actions
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    stage_node = launch_ros.actions.Node(
        package='stage_ros2',
        executable='stage_ros2',
        name='model',
        parameters=[
            {
                '/use_sim_time': True
            },
            {
                'world_file': launch.substitutions.LaunchConfiguration('world', default=get_package_share_directory('cart_launch') + '/stage_worlds/empty.world')
            }
        ]
    )
    stage_controller = launch_ros.actions.Node(
        package='stage_controller',
        executable='stage_controller',
        name='robot',
        output='log',
        parameters=[
            {
                '/use_sim_time': True
            },
            {
                'length': 1.5
            },
            {
                'max_steering': 0.5
            },
            {
                'max_steering_rate': 1.0
            },
            {
                'max_velocity': 18.0
            },
            {
                'max_acc': 2.0
            }
        ]
    )
    stage_throttle = launch_ros.actions.Node(
        package='stage_controller',
        executable='stage_throttle',
        name='robot',
        output='log',
        parameters=[
            {
                '/use_sim_time': True
            },
            {
                'length': 1.5
            },
            {
                'max_steering': 0.5
            },
            {
                'max_steering_rate': 1.0
            },
            {
                'max_velocity': 18.0
            },
            {
                'max_throttle': 400.0
            },
            {
                'max_throttle_rate': 800.0
            },
            {
                'max_acc': 2.0
            },
            {
                'velocity_noise': launch.substitutions.LaunchConfiguration('velocity_noise')
            },
            {
                'mass': 500.0
            },
            {
                'friction': 20.0
            },
            {
                'wind_friction': 0.1
            },
            {
                'brake': 15.0
            },
            {
                'throttle': 50.0
            },
            {
                'exp': 0.4
            }
        ]
    )

    return launch.LaunchDescription([
        launch.actions.DeclareLaunchArgument(
            'world',
            default_value=get_package_share_directory(
                'cart_launch') + '/stage_worlds/empty.world'
        ),
        launch.actions.DeclareLaunchArgument(
            'control_velocity',
            default_value = 'true'
        ),
        launch.actions.DeclareLaunchArgument(
            'velocity_noise',
            default_value = '0.0'
        ),
        stage_node,
        stage_controller,
        # stage_throttle,
    ])


# if __name__ == '__main__':
#     generate_launch_description()
