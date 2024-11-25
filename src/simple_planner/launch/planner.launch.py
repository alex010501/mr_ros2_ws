from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package="stage_ros",
            executable="stageros",
            name="model",
            arguments=["/path/to/simple.world"],
            remappings=[
                ("/odom", "/robot/odom"),
                ("/base_pose_ground_truth", "/robot/base_pose_ground_truth")
            ]
        ),
        Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            name="map_to_odom_transform_pub",
            arguments=["0", "0", "0", "0", "0", "0", "map", "odom"]
        ),
        Node(
            package="map_server",
            executable="map_server",
            name="map_server",
            arguments=["/path/to/cave.yaml"]
        ),
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
        Node(
            package="rviz2",
            executable="rviz2",
            name="rviz",
            arguments=["--display-config", "/path/to/planner.rviz"],
            output="screen"
        )
    ])
