from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    return LaunchDescription([
        Node(package="foxglove_bridge",
             executable="foxglove_bridge",
             name="foxglove_bridge",
             output="screen"),
        Node(package="path_planner_ros2",
             executable="a_star_node",
             name="a_star_node",
             output="screen"),
    ])
