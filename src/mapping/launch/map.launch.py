from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    # Аргументы для мирового файла
    world_arg = DeclareLaunchArgument(
        name='world',
        default_value=PathJoinSubstitution(
            [FindPackageShare('cart_launch'), 'stage_worlds', 'mapping.world']
        ),
        description='Stage world file'
    )

    # Включение файла запуска из пакета cart_launch
    cart_stage_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution(
            [FindPackageShare('cart_launch'), 'launch', 'cart_stage.launch.py']
        )),
        launch_arguments={'world': LaunchConfiguration('world')}.items(),
    )

    # Узел simple_controller
    controller_node = Node(
        package='simple_controller',
        executable='controller_node',
        name='controller',
        output='log',
        parameters=[
            PathJoinSubstitution(
                [FindPackageShare('simple_controller'), 'launch', 'controller.yaml']
            )
        ],
        remappings=[
            ('/controller/simple_controller/ground_truth', '/robot/base_pose_ground_truth'),
            ('/controller/simple_controller/odom', '/robot/odom'),
            ('steering', '/robot/steering'),
        ]
    )

    # Узел для отправки команды скорости
    # velocity_node = Node(
    #     package='ros2topic',
    #     executable='pub',
    #     name='vel_node',
    #     arguments=['/robot/velocity', 'std_msgs/Float32', '2.0', '-r1'],
    #     output='screen'
    # )
    velocity_node = Node(
        package='vel_pub',
        executable='velocity_publisher',
        name='vel_node',
        output='screen'
    )

    # Узел карты
    map_node = Node(
        package='mapping',
        executable='mapping',
        name='map',
        output='screen',
        remappings=[
            ('scan', 'base_scan'),
        ]
    )

    # Узел rviz
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz',
        arguments=['--display-config', PathJoinSubstitution(
            [FindPackageShare('mapping'), 'rviz', 'map.rviz']
        )],
        output='screen'
    )

    return LaunchDescription([
        world_arg,
        cart_stage_launch,
        controller_node,
        velocity_node,
        map_node,
        rviz_node
    ])