cmake_minimum_required(VERSION 3.8)
project(bringup)

# 设置策略以抑制警告
if(POLICY CMP0074)
  cmake_policy(SET CMP0074 NEW)
endif()

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

# 如果支持，添加编译器警告
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# ============ 查找依赖包 ============
find_package(ament_cmake REQUIRED)

# ROS2核心依赖
find_package(rclcpp REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
# ============ 添加PCL依赖 ============
find_package(PCL REQUIRED)
if(PCL_FOUND)
  message(STATUS "PCL found: version ${PCL_VERSION}")
  # 包含PCL头文件路径
  include_directories(${PCL_INCLUDE_DIRS})
endif()

# ============ 添加pcl_conversions ============
find_package(pcl_conversions REQUIRED)

# ============ 添加Eigen3（用于矩阵运算）============
find_package(Eigen3 REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIRS})


# ============ 添加点云过滤节点 ============
add_executable(self_filter_node
  src/self_filter_node.cpp  # 这是独立的文件
)
add_executable(tf_diff_calculator
  src/tf_diff_calculator.cpp  # 这是独立的文件
)
target_link_libraries(self_filter_node
  ${rclcpp_LIBRARIES}
  ${tf2_ros_LIBRARIES}
  ${geometry_msgs_LIBRARIES}
  ${tf2_LIBRARIES}
  ${sensor_msgs_LIBRARIES}
  ${tf2_geometry_msgs_LIBRARIES}
  ${PCL_LIBRARIES}
  ${pcl_conversions_LIBRARIES}
)
target_link_libraries(tf_diff_calculator
  ${rclcpp_LIBRARIES}
  ${tf2_ros_LIBRARIES}
  ${geometry_msgs_LIBRARIES}
  ${tf2_LIBRARIES}
  ${tf2_geometry_msgs_LIBRARIES}
)
ament_target_dependencies(self_filter_node
  rclcpp
  tf2_ros
  geometry_msgs
  tf2
  sensor_msgs
  tf2_geometry_msgs
  pcl_conversions
)

ament_target_dependencies(tf_diff_calculator
  rclcpp
  tf2
  tf2_ros
  tf2_geometry_msgs
  geometry_msgs
)

install(TARGETS
  self_filter_node tf_diff_calculator
  DESTINATION lib/${PROJECT_NAME}
)

# ============ 安装文件 ============
install(
  DIRECTORY behavior_tree config launch map rviz 
  DESTINATION share/${PROJECT_NAME}
)


# ============ 导出依赖 ============
ament_export_dependencies(
  rclcpp
  tf2_ros
  geometry_msgs
  tf2
  sensor_msgs
  tf2_geometry_msgs
  pcl_conversions
  std_msgs
)

ament_package()