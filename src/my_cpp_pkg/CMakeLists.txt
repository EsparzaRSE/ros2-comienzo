cmake_minimum_required(VERSION 3.8)
project(my_cpp_pkg)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(example_interfaces REQUIRED)
find_package(my_robot_interfaces REQUIRED)

# crear el ejecutable e instalar el nodo
add_executable(cpp_node src/my_first_node.cpp)
ament_target_dependencies(cpp_node rclcpp)

add_executable(robot_news_station src/robot_news_station.cpp)
ament_target_dependencies(robot_news_station rclcpp example_interfaces)

add_executable(smartphone src/smartphone.cpp)
ament_target_dependencies(smartphone rclcpp example_interfaces)

add_executable(number_publisher src/actividades/actividad2/number_publisher.cpp)
ament_target_dependencies(number_publisher rclcpp example_interfaces)

add_executable(number_counter src/actividades/actividad2/number_counter.cpp)
ament_target_dependencies(number_counter rclcpp example_interfaces)

add_executable(add_two_ints_server src/add_two_ints_server.cpp)
ament_target_dependencies(add_two_ints_server rclcpp example_interfaces)

add_executable(add_two_ints_client src/add_two_ints_client.cpp)
ament_target_dependencies(add_two_ints_client rclcpp example_interfaces)

add_executable(number_publisher3 src/actividades/actividad3/number_publisher3.cpp)
ament_target_dependencies(number_publisher3 rclcpp example_interfaces)

add_executable(number_counter3 src/actividades/actividad3/number_counter3.cpp)
ament_target_dependencies(number_counter3 rclcpp example_interfaces)

add_executable(call_reset_counter3 src/actividades/actividad3/call_reset_counter3.cpp)
ament_target_dependencies(call_reset_counter3 rclcpp example_interfaces)

add_executable(hardware_status_publisher src/hw_status_publisher.cpp)
ament_target_dependencies(hardware_status_publisher rclcpp my_robot_interfaces)

add_executable(battery src/actividades/actividad4/battery.cpp)
ament_target_dependencies(battery rclcpp my_robot_interfaces)

add_executable(led_panel src/actividades/actividad4/led_panel.cpp)
ament_target_dependencies(led_panel rclcpp my_robot_interfaces)

add_executable(number_publisher_params src/number_publisher_params.cpp)
ament_target_dependencies(number_publisher_params rclcpp example_interfaces)

add_executable(robot_news_station_params src/actividades/actividad5/robot_news_station_params.cpp)
ament_target_dependencies(robot_news_station_params rclcpp example_interfaces)

add_executable(led_panel_params src/actividades/actividad5/led_panel_params.cpp)
ament_target_dependencies(led_panel_params rclcpp my_robot_interfaces)

install(TARGETS 
  cpp_node
  robot_news_station
  smartphone
  number_publisher
  number_counter
  add_two_ints_server
  add_two_ints_client
  number_publisher3
  number_counter3
  call_reset_counter3
  hardware_status_publisher
  battery 
  led_panel
  number_publisher_params
  robot_news_station_params
  led_panel_params
  DESTINATION lib/${PROJECT_NAME}
)

#tests
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
