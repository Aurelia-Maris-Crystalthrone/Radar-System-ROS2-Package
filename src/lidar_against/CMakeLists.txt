cmake_minimum_required(VERSION 3.8)
project(lidar_against)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)

add_executable(base_point_publisher src/base_point_publisher.cpp)
ament_target_dependencies(base_point_publisher rclcpp geometry_msgs)
add_executable(radar_distance src/radar_distance.cpp)
ament_target_dependencies(radar_distance rclcpp geometry_msgs std_msgs)
add_executable(radar_position_publisher src/radar_position_publisher.cpp)
ament_target_dependencies(radar_position_publisher rclcpp geometry_msgs)
add_executable(distance_monitor src/distance_monitor.cpp)
ament_target_dependencies(distance_monitor rclcpp std_msgs)

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

install(TARGETS
  radar_position_publisher
  base_point_publisher
  radar_distance
  distance_monitor
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY launch
DESTINATION share/${PROJECT_NAME})

ament_package()
