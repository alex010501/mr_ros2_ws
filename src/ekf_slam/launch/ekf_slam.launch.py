from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    # Аргумент для задания мира
    world_arg = DeclareLaunchArgument(
        'world',
        default_value=PathJoinSubstitution([
            FindPackageShare('cart_launch'), 'stage_worlds', 'kalman_map.world'
        ]),
        description='Path to the stage world file'
    )

    # Узел EKF SLAM
    ekf_slam_node = Node(
        package='ekf_slam',
        executable='ekf_slam',
        name='ekf_slam',
        output='screen',
        remappings=[
            ('/scan', '/base_scan'),
            ('/odom', '/robot/odom')
        ]
    )

    # Включение запуска симуляции Stage
    cart_stage_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution([
            FindPackageShare('cart_launch'), 'launch', 'cart_stage.launch.py'
        ])),
        launch_arguments={'world': LaunchConfiguration('world')}.items()
    )

    # Узел простого контроллера
    controller_node = Node(
        package='simple_controller',
        executable='controller_node',
        name='controller',
        output='log',
        parameters=[
            PathJoinSubstitution([
                FindPackageShare('simple_controller'), 'launch', 'controller.yaml'
            ])
        ],
        remappings=[
            ('/controller/simple_controller/ground_truth', '/robot/base_pose_ground_truth'),
            ('/controller/simple_controller/odom', '/robot/odom'),
            ('steering', '/robot/steering')
        ]
    )

    # Узел публикации скорости
    vel_pub_node = Node(
        package='rostopic',
        executable='rostopic',
        name='vel_node',
        arguments=['pub', '/robot/velocity', 'std_msgs/Float32', '2.0', '-r1']
    )

    # Узел RViz
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz',
        arguments=['--display-config', PathJoinSubstitution([
            FindPackageShare('barrel_slam'), 'launch', 'slam.rviz'
        ])]
    )

    return LaunchDescription([
        world_arg,
        ekf_slam_node,
        cart_stage_launch,
        controller_node,
        vel_pub_node,
        rviz_node
    ])
