import os
import sys

import launch
import launch_ros.actions
from ament_index_python.packages import get_package_share_directory


# REQUIRES FIXING


def generate_launch_description():
    ld = launch.LaunchDescription([
        launch.actions.DeclareLaunchArgument(
            name='world_name',
            default_value='simple.world'
        ),
        launch.actions.DeclareLaunchArgument(
            name='gui',
            default_value='true'
        ),
        launch_ros.actions.Node(
            package='gazebo_ros',
            executable='spawn_model',
            name='spawn_urdf',
            output='screen'
        ),
        launch_ros.actions.Node(
            package='tf',
            executable='static_transform_publisher',
            name='camera_tf_pub'
        ),
        launch_ros.actions.Node(
            package='tf',
            executable='static_transform_publisher',
            name='lidar_tf_pub'
        ),
        launch_ros.actions.Node(
            package='tf',
            executable='static_transform_publisher',
            name='chassis_tf_pub'
        ),
        launch.actions.IncludeLaunchDescription(
            launch.launch_description_sources.PythonLaunchDescriptionSource(
                os.path.join(get_package_share_directory(
                    'gazebo_ros'), 'launch/empty_world.launch.py')
            ),
            launch_arguments={
                'world_name': launch.substitutions.LaunchConfiguration('world_name'),
                'verbose': 'true',
                'physics': 'ode',
                'gui': launch.substitutions.LaunchConfiguration('gui')
            }.items()
        )
    ])
    return ld


# if __name__ == '__main__':
#     generate_launch_description()
