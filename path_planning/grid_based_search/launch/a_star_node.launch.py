from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    # Foxglove WebSocket data source URL with layout
    foxglove_url = "foxglove://open?ds=foxglove-websocket&ds.url=ws://localhost:8765"

    return LaunchDescription([
        ExecuteProcess(cmd=["foxglove-studio", foxglove_url], output="screen"),
        Node(package="foxglove_bridge",
             executable="foxglove_bridge",
             name="foxglove_bridge",
             output="screen"),
        Node(package="grid_based_search",
             executable="a_star_node",
             name="a_star_node",
             output="screen"),
    ])
