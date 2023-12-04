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

# crear el ejecutable e instalar el nodo
add_executable(tortuga_spawner src/tortuga_spawner.cpp)
ament_target_dependencies(tortuga_spawner rclcpp turtlesim my_robot_interfaces)

install(TARGETS 
  tortuga_spawner
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
