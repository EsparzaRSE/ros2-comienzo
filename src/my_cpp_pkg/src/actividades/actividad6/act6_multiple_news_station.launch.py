from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    ld = LaunchDescription()

    robot_news_station_giskard_node = Node(
        package="my_cpp_pkg",
        executable="robot_news_station_params",
        name="robot_news_station_giskard",
        parameters=[ {"robot_name": "R. Giskard Reventlov"} ]
    )
    robot_news_station_bb8_node = Node(
        package="my_cpp_pkg",
        executable="robot_news_station_params",
        name="robot_news_station_bb8",
        parameters=[ {"robot_name": "BB-8"} ]
    )
    robot_news_station_daneel_node = Node(
        package="my_cpp_pkg",
        executable="robot_news_station_params",
        name="robot_news_station_daneel",
        parameters=[ {"robot_name": "R. Daneel Olivaw"} ]
    )
    robot_news_station_jander_node = Node(
        package="my_cpp_pkg",
        executable="robot_news_station_params",
        name="robot_news_station_jander",
        parameters=[ {"robot_name": "Jander Panell"} ]
    )
    robot_news_station_c3po_node = Node(
        package="my_cpp_pkg",
        executable="robot_news_station_params",
        name="robot_news_station_c3po",
        parameters=[ {"robot_name": "C3PO"} ]
    )
    smartphone_node = Node(
        package="my_cpp_pkg",
        executable="smartphone",
    )

    ld.add_action(robot_news_station_giskard_node)
    ld.add_action(robot_news_station_bb8_node)
    ld.add_action(robot_news_station_daneel_node)
    ld.add_action(robot_news_station_jander_node)
    ld.add_action(robot_news_station_c3po_node)
    ld.add_action(smartphone_node)

    return ld