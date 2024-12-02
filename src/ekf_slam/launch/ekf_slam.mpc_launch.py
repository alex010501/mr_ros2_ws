from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.actions import ExecuteProcess
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # Get package directories
    slam_dir = get_package_share_directory('ekf_slam')
    mpc_package_dir = get_package_share_directory('mpc_controller')
    cart_launch_dir = get_package_share_directory('cart_launch')

    # Paths to configuration files
    param_file_path = os.path.join(mpc_package_dir, 'config', 'controller.yaml')
    rqt_perspective_path = os.path.join(mpc_package_dir, 'rqt', 'steer_error.perspective')
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

    delayed_mpc_node = TimerAction(
        period = 1.0,  # Delay in seconds
        actions = [
            Node(
                package='mpc_controller',
                executable='mpc_controller',
                name='controller',
                parameters=[{'use_sim_time': True}, param_file_path],
                output='screen'
            )
        ]
    )

    return LaunchDescription([
        # Include cart_stage.launch.py
        cart_stage_launch,

        # Node for the MPC controller
        delayed_mpc_node,
        
        Node(
            package='mapping',
            executable='mapping',
            name='map',
            output='screen',
            remappings=[
                ('scan', 'base_scan'),
            ]
        ),

        Node(
            package='ekf_slam',
            executable='ekf_slam',
            name='ekf_slam',
            output='screen',
            remappings=[
                ('/scan', '/base_scan')
            ]
        ),

        # ExecuteProcess to launch RQT
        ExecuteProcess(
            cmd=['rqt', '--perspective-file', rqt_perspective_path],
            name='rqt',
            output='screen',
        ),

        # ExecuteProcess to launch RViz
        ExecuteProcess(
            cmd=['rviz2', '-d', rviz_config_path],
            name='rviz2',
            output='screen',
        )
    ])