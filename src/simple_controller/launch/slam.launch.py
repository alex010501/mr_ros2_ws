import launch
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable, IncludeLaunchDescription, LogInfo
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration, FindPackageShare
from launch.conditions import IfCondition

def generate_launch_description():
    # Декларация аргументов
    world_name = LaunchConfiguration('world_name', default='$(find cart_launch)/worlds/slam.world')

    # Создание описания запуска
    return LaunchDescription([
        # Установка переменной окружения для файла конфигурации ROS Console
        SetEnvironmentVariable(
            'ROSCONSOLE_CONFIG_FILE',
            [FindPackageShare('simple_controller'), '/config/rosconsole.config']
        ),
        
        # Объявление аргумента для имени мира
        DeclareLaunchArgument('world_name', default_value='$(find cart_launch)/worlds/slam.world'),
        
        # Включение другого launch файла для старта cart в Gazebo
        IncludeLaunchDescription(
            FindPackageShare('cart_launch') + '/launch/cart.launch',
            launch_arguments={'world_name': world_name}.items()
        ),
        
        # Запуск узла odo2tf для преобразования сообщений о базовой позе в tf
        Node(
            package='odo2tf',
            executable='odo2tf',
            name='odo2tf',
            output='screen',
            remappings=[('/odo', '/robot/base_pose_ground_truth')]
        ),
        
        # Запуск узла контроллера
        Node(
            package='simple_controller',
            executable='simple_controller',
            name='controller',
            output='screen',
            parameters=[{'command': 'load', 'file': FindPackageShare('simple_controller') + '/launch/controller.yaml', 'ns': 'simple_controller'}],
            remappings=[
                ('/controller/simple_controller/ground_truth', '/robot/base_pose_ground_truth'),
                ('/controller/simple_controller/odom', '/robot/odom'),
                ('steering', '/robot/steering')
            ]
        ),
        
        # Условный запуск узла rqt для отображения
        Node(
            package='simple_controller',
            executable='start_rqt',
            name='rqt',
            arguments=['--perspective-file', FindPackageShare('simple_controller') + '/cfg/steer_error.perspective'],
            output='screen'
        ),
        
        # Запуск gzclient через скрипт
        Node(
            package='simple_controller',
            executable='start_gzclient',
            name='gzclient',
            output='screen'
        )
    ])
