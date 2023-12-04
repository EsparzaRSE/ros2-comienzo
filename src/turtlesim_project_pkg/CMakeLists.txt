cmake_minimum_required(VERSION 3.8)
project(turtlesim_project_pkg)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(my_robot_interfaces REQUIRED)
find_package(turtlesim REQUIRED)
find_package(geometry_msgs REQUIRED)

# crear el ejecutable e instalar el nodo
add_executable(tortuga_spawner src/tortuga_spawner.cpp)
ament_target_dependencies(tortuga_spawner rclcpp turtlesim my_robot_interfaces)

add_executable(tortuga_controller src/tortuga_controller.cpp)
ament_target_dependencies(tortuga_controller rclcpp turtlesim my_robot_interfaces geometry_msgs)

install(TARGETS 
  tortuga_spawner
  tortuga_controller
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
