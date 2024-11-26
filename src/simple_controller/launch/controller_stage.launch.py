import os
import sys

import launch
import launch_ros.actions
from ament_index_python.packages import get_package_share_directory
import yaml

def load_yaml(package_name, file_path):
    package_path = get_package_share_directory(package_name)
    full_path = os.path.join(package_path, file_path)
    
    try:
        with open(full_path, 'r') as f:
            return yaml.safe_load(f)
    except Exception as e:
        raise RuntimeError(f'Failed to load {full_path}: {str(e)}')

def generate_launch_description():

    controller_config = load_yaml('simple_controller', 'launch/controller.yaml')

    ld = launch.LaunchDescription([
        launch.actions.DeclareLaunchArgument(
            name='control_velocity',
            default_value='true'
        ),
        launch.actions.DeclareLaunchArgument(
            name='velocity_noise',
            default_value='0.0'
        ),
        launch_ros.actions.Node(
            package='simple_controller',
            executable='controller_node',
            name='controller',
            output='screen',
            parameters=[
                {'proportional': controller_config['proportional']},
                {'differential': controller_config['differential']},
                {'integral': controller_config['integral']},
                {'max_antiwindup_error': controller_config['max_antiwindup_error']},
                {'radius': controller_config['radius']},
                {'cy': controller_config['cy']},
                {'traj_length': controller_config['traj_length']},
                {'timer_period': controller_config['timer_period']}
            ]
        ),
        launch.actions.IncludeLaunchDescription(
            launch.launch_description_sources.PythonLaunchDescriptionSource(
                os.path.join(get_package_share_directory(
                    'cart_launch'), 'launch/cart_stage.launch.py')
            ),
            launch_arguments={
                'control_velocity': launch.substitutions.LaunchConfiguration('control_velocity'),
                'velocity_noise': launch.substitutions.LaunchConfiguration('velocity_noise')
            }.items()
        )
    ])
    return ld


# if __name__ == '__main__':
#     generate_launch_description()
