import os
import sys

import launch
import launch_ros.actions
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    ld = launch.LaunchDescription([
        launch_ros.actions.Node(
            package='simple_controller',
            executable='controller_node',
            name='controller',
            output='screen'
        ),
        # launch_ros.actions.Node(
        #     package='rostopic',
        #     executable='rostopic',
        #     name='vel_node'
        # ),
        launch_ros.actions.Node(
            package='feature_matcher',
            executable='feature_matcher',
            name='matcher',
            output='screen'
        ),
        launch_ros.actions.Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', os.path.join(get_package_share_directory(
                'feature_matcher'), 'launch/matcher.rviz')]
        ),
        launch.actions.IncludeLaunchDescription(
            launch.launch_description_sources.PythonLaunchDescriptionSource(
                os.path.join(get_package_share_directory(
                    'cart_launch'), 'launch/cart_stage.launch.py')
            ),
            launch_arguments={
                'world': get_package_share_directory('cart_launch') + '/stage_worlds/feature_map_simple.world'
            }.items()
        )
    ])
    return ld


# if __name__ == '__main__':
#     generate_launch_description()
