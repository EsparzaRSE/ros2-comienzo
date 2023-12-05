cmake_minimum_required(VERSION 3.8)
project(intento2_turtlesim_project_pkg)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(turtlesim REQUIRED)
find_package(my_robot_interfaces REQUIRED)
find_package(geometry_msgs REQUIRED)

include_directories(
  ${CMAKE_CURRENT_SOURCE_DIR}/include
)

# crear el ejecutable e instalar el nodo
add_executable(tortuga_spawner2 src/tortuga_spawner2.cpp)
ament_target_dependencies(tortuga_spawner2 rclcpp turtlesim my_robot_interfaces)

add_executable(tortuga_controller2 src/tortuga_controller2.cpp)
ament_target_dependencies(tortuga_controller2 rclcpp turtlesim my_robot_interfaces geometry_msgs)

install(TARGETS 
  tortuga_spawner2
  tortuga_controller2
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
