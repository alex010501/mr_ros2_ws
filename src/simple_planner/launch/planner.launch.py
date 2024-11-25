from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import PathJoinSubstitution, FindPackageShare
from launch_ros.actions import Node

def generate_launch_description():
    # Путь к stage_worlds
    stage_worlds_path = PathJoinSubstitution(
        [FindPackageShare("cart_launch"), "stage_worlds"]
    )

    # Создание описания запуска
    return LaunchDescription([
        # Параметр симуляции времени
        DeclareLaunchArgument(
            "use_sim_time",
            default_value="true",
            description="Use simulation time"
        ),

        # Узел Stage
        Node(
            package="stage_ros",
            executable="stageros",
            name="model",
            arguments=[PathJoinSubstitution([stage_worlds_path, "simple.world"])],
            remappings=[
                ("/odom", "/robot/odom"),
                ("/base_pose_ground_truth", "/robot/base_pose_ground_truth")
            ]
        ),

        # Статическая трансформация map -> odom
        Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            name="map_to_odom_transform_pub",
            arguments=["0", "0", "0", "0", "0", "0", "map", "odom", "1000"]
        ),

        # Сервер карты
        Node(
            package="nav2_map_server",
            executable="map_server",
            name="map_server",
            parameters=[{"yaml_filename": PathJoinSubstitution([stage_worlds_path, "cave.yaml"])}]
        ),

        # Узел Planner
        Node(
            package="simple_planner",
            executable="simple_planner",
            name="planner",
            output="screen",
            remappings=[
                ("/planner/target_pose", "/move_base_simple/goal"),
                ("/planner/ground_truth", "/robot/base_pose_ground_truth")
            ]
        ),

        # Узел RViz
        Node(
            package="rviz2",
            executable="rviz2",
            name="rviz",
            output="screen",
            arguments=[
                "--display-config", PathJoinSubstitution(
                    [FindPackageShare("simple_planner"), "launch", "planner.rviz"]
                )
            ]
        ),
    ])
