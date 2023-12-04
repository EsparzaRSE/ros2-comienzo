from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    ld = LaunchDescription()

    turtlesim_node = Node(
        package="turtlesim",
        executable="turtlesim_node",
        name="mi_tortuga"
    )

    tortuga_spawner_node = Node(
        package="turtlesim_project_pkg",
        executable="tortuga_spawner",
        parameters=[
            {"turtle_name_prefix": "NinjaTurtle"}
        ],
    )
    tortuga_controller_node = Node(
        package="turtlesim_project_pkg",
        executable="tortuga_controller",
    )

    ld.add_action(turtlesim_node)
    ld.add_action(tortuga_spawner_node)
    ld.add_action(tortuga_controller_node)

    return ld