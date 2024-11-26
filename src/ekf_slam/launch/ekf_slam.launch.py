from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.actions import ExecuteProcess
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # Get package directories
    slam_dir = get_package_share_directory('ekf_slam')
    simple_controller_dir = get_package_share_directory('simple_controller')
    cart_launch_dir = get_package_share_directory('cart_launch')

    # Paths to configuration files
    param_file_path = os.path.join(simple_controller_dir, 'launch', 'controller.yaml')
    rviz_config_path = os.path.join(slam_dir, 'launch', 'slam.rviz')

    # Include cart_stage launch
    cart_stage_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(cart_launch_dir, 'launch', 'cart_stage.launch.py')),
        launch_arguments={
            'world': os.path.join(cart_launch_dir, 'stage_worlds', 'kalman_map.world'),
            'control_velocity': 'true',
            'velocity_noise': '0.0'
        }.items()
    )

    return LaunchDescription([
        cart_stage_launch,

        Node(
            package='simple_controller',
            executable='controller_node',
            name='controller',
            output='log',
            parameters=[param_file_path],
            remappings=[
                ('/controller/simple_controller/ground_truth', '/robot/base_pose_ground_truth'),
                ('/controller/simple_controller/odom', '/robot/odom'),
                ('steering', '/robot/steering'),
            ]
        ),

        Node(
            package='vel_pub',
            executable='velocity_publisher',
            name='vel_node',
            output='screen'
        ),

        Node(
            package='ekf_slam',
            executable='ekf_slam',
            name='ekf_slam',
            output='screen',
            remappings=[
                ('/scan', '/base_scan'),
                ('/odom', '/robot/odom')
            ]
        ),

        ExecuteProcess(
            cmd=['rviz2', '-d', rviz_config_path],
            name='rviz2',
            output='screen',
        )
    ])