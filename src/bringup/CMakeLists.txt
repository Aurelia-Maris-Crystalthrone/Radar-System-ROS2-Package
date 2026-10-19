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
  include_directories(${PCL_INCLUDE_DIRS})
endif()

# ============ 添加pcl_conversions ============
find_package(pcl_conversions REQUIRED)

# ============ 添加Eigen3（用于矩阵运算）============
find_package(Eigen3 REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIRS})

# ============ 桥接头文件搜索路径 ============
# 注意：原实现写的是 target_include_directories(${PROJECT_NAME} ...)，但本包没有任何
# 名为 bringup 的 target，CMake 配置阶段就会报 "Cannot specify include directories
# for target \"bringup\" which is not built by this project"。这里改为显式挂在用到
# 头文件的目标上。
set(BRINGUP_INCLUDE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/include)

# ============ 添加点云相关节点 ============
add_executable(self_filter_node
  src/self_filter_node.cpp
)
target_include_directories(self_filter_node PRIVATE ${BRINGUP_INCLUDE_DIR})
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
ament_target_dependencies(self_filter_node
  rclcpp
  tf2_ros
  geometry_msgs
  tf2
  sensor_msgs
  tf2_geometry_msgs
  pcl_conversions
)

add_executable(tf_diff_calculator
  src/tf_diff_calculator.cpp
)
target_include_directories(tf_diff_calculator PRIVATE ${BRINGUP_INCLUDE_DIR})
ament_target_dependencies(tf_diff_calculator
  rclcpp
  tf2
  tf2_ros
  tf2_geometry_msgs
  geometry_msgs
)

add_executable(radar_to_serial_node src/radar_to_serial_node.cpp)
target_include_directories(radar_to_serial_node PRIVATE ${BRINGUP_INCLUDE_DIR})
ament_target_dependencies(radar_to_serial_node
  rclcpp
  std_msgs
  geometry_msgs
)

# ========== 内嵌 serial 库 ==========
# 目标名不能再用 `serial`：一旦系统里装了 ros-humble-serial，`find_package(serial)`
# 导出的 target 会与 add_library(serial) 撞名；这里统一改名 serial_vendor，并去掉
# 那个多余的 find_package(serial REQUIRED)。
set(SERIAL_SRC_DIR ${CMAKE_CURRENT_SOURCE_DIR}/lib/serial)

add_library(serial_vendor STATIC
  ${SERIAL_SRC_DIR}/src/serial.cc
  ${SERIAL_SRC_DIR}/src/impl/unix.cc
  ${SERIAL_SRC_DIR}/src/impl/list_ports/list_ports_linux.cc
)

target_include_directories(serial_vendor PUBLIC
  $<BUILD_INTERFACE:${SERIAL_SRC_DIR}/include>
)

find_package(Threads REQUIRED)
target_link_libraries(serial_vendor PRIVATE Threads::Threads udev)
target_compile_features(serial_vendor PUBLIC cxx_std_11)

target_link_libraries(radar_to_serial_node serial_vendor)
# ====================================

install(TARGETS
  self_filter_node tf_diff_calculator radar_to_serial_node
  DESTINATION lib/${PROJECT_NAME}
)

# ============ 安装文件 ============
install(
  DIRECTORY behavior_tree config map launch rviz lib
  DESTINATION share/${PROJECT_NAME}
)
install(DIRECTORY include/
  DESTINATION include/${PROJECT_NAME}
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
ament_export_include_directories(include)

ament_package()
