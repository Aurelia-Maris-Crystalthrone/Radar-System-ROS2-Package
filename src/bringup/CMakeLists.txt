cmake_minimum_required(VERSION 3.8)
project(bringup)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(nav_msgs REQUIRED)

add_executable(base_to_robot src/base_to_robot.cpp)
ament_target_dependencies(base_to_robot
  rclcpp
  tf2_ros
  tf2_geometry_msgs
  geometry_msgs
  std_msgs
)
add_executable(odom_to_tf src/odom_to_tf.cpp)
ament_target_dependencies(odom_to_tf
  
  rclcpp
  nav_msgs
  tf2_ros
  geometry_msgs
)
install(TARGETS
  base_to_robot odom_to_tf
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY launch map rviz config behavior_tree
  DESTINATION share/${PROJECT_NAME}
)


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

ament_export_dependencies(
  rclcpp
  tf2_ros
  geometry_msgs
  tf2
  sensor_msgs
  tf2_geometry_msgs
  std_msgs
)

ament_package()
