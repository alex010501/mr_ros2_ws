import launch
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo, IncludeLaunchDescription, SetEnvironmentVariable
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch_ros.substitutions import FindPackageShare
from launch.conditions import IfCondition

def generate_launch_description():
    # Декларация аргументов
    control_velocity = LaunchConfiguration('control_velocity', default='true')
    rqt_persp = LaunchConfiguration('rqt_persp', default='$(find simple_controller)/cfg/steer_error.perspective')
    velocity_noise = LaunchConfiguration('velocity_noise', default='0.0')

    # Создание описания запуска
    return LaunchDescription([
        # Установка переменной окружения для файла конфигурации ROS Console
        SetEnvironmentVariable(
            'ROSCONSOLE_CONFIG_FILE',
            [FindPackageShare('simple_controller'), '/config/rosconsole.config']
        ),
        
        # Объявление аргументов
        DeclareLaunchArgument('control_velocity', default_value='true'),
        DeclareLaunchArgument('rqt_persp', default_value='$(find simple_controller)/cfg/steer_error.perspective'),
        DeclareLaunchArgument('velocity_noise', default_value='0.0'),
        
        # Включение другого launch файла для старта cart в Stage World
        IncludeLaunchDescription(
            FindPackageShare('cart_launch') + '/launch/cart_stage.launch',
            launch_arguments={'control_velocity': control_velocity, 'velocity_noise': velocity_noise}.items()
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
        
        # Условный запуск узла rqt, если параметр rqt_persp не пустой
        Node(
            package='simple_controller',
            executable='start_rqt',
            name='rqt',
            arguments=['--perspective-file', rqt_persp],
            output='log',
            condition=IfCondition(rqt_persp)
        )
    ])
