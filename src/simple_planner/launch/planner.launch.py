import os
import sys

import launch
import launch_ros.actions
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    lifecycle_nodes = ['map_server']
    use_sim_time = True
    autostart = True

    ld = launch.LaunchDescription([
        launch_ros.actions.Node(
            package='stage_ros2',
            executable='stage_ros2',
            name='model',
            parameters=[
                {
                    '/use_sim_time': True
                },
                {
                    'world_file': get_package_share_directory('cart_launch') + '/stage_worlds/simple.world'
                }
            ]
        ),

        launch_ros.actions.Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            output="screen" ,
            arguments=["0", "0", "0", "0", "0", "0", "map", "odom"]
        ),
    
        launch_ros.actions.Node(
                package='nav2_map_server',
                executable='map_server',
                output='screen',
                parameters=[
                    {
                        "yaml_filename": get_package_share_directory('cart_launch') + '/stage_worlds/cave.yaml'
                    }
                ]
        ),

        launch_ros.actions.Node(
                package='nav2_lifecycle_manager',
                executable='lifecycle_manager',
                name='lifecycle_manager',
                output='screen',
                emulate_tty=True,
                parameters=[{'use_sim_time': use_sim_time},
                            {'autostart': autostart},
                            {'node_names': lifecycle_nodes}]
        ),

        launch_ros.actions.Node(
            package='simple_planner',
            executable='simple_planner',
            name='planner',
            output='screen',
            parameters=[
                {
                    '/use_sim_time': True
                }
            ]
        ),
        launch_ros.actions.Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            parameters=[
                {
                    '/use_sim_time': True
                }
            ],
            arguments=['-d', get_package_share_directory('simple_planner') + '/launch/planner.rviz']
        )
    ])
    return ld


# if __name__ == '__main__':
#     generate_launch_description()
