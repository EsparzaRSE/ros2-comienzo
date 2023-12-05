from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    ld = LaunchDescription()

    turtlesim_node = Node(
        package="turtlesim",
        executable="turtlesim_node",
        name="mi_tortuga"
    )

    tortuga_spawner2_node = Node(
        package="intento2_turtlesim_project_pkg",
        executable="tortuga_spawner2",
        parameters=[
            {"turtle_name_prefix": "NinjaTurtle"}
        ],
    )

    tortuga_controller2_node = Node(
        package="intento2_turtlesim_project_pkg",
        executable="tortuga_controller2",
    )

    ld.add_action(turtlesim_node)
    ld.add_action(tortuga_spawner2_node)
    ld.add_action(tortuga_controller2_node)

    return ld