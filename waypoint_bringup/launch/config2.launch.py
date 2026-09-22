from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    ld = LaunchDescription()
    turtle_controller = Node(
    package="turtle_handler",
    executable="turtle_controller"
    )
    mission_client = Node(
    package="turtle_hanler",
    executable="mission_client"
    )
    turtlesim_node = Node(
        package="turtlesim",
        executable="turtlesim_node"
    )
    ld.add_action(turtle_controller)
    ld.add_action(mission_client)
    ld.add_action(turtlesim_node)
    return ld