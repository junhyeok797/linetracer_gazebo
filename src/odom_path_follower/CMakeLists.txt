cmake_minimum_required(VERSION 3.8)
project(odom_path_follower)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(ament_index_cpp REQUIRED)

include_directories(
  include/odom_path_follower
)

add_executable(ekf_odom
  src/ekf_odom_node.cpp
  src/ekf_odom.cpp
)
target_include_directories(ekf_odom PRIVATE
  ${EIGEN3_INCLUDE_DIR}
)
ament_target_dependencies(ekf_odom
  rclcpp
  geometry_msgs
  nav_msgs
  sensor_msgs
  tf2_ros
  tf2_geometry_msgs
)

add_executable(path_follower
  src/path_follower_node.cpp
  src/path_follower.cpp
)
ament_target_dependencies(path_follower
  rclcpp
  nav_msgs
  geometry_msgs
  tf2
  tf2_geometry_msgs
  ament_index_cpp
)

install(TARGETS ekf_odom path_follower
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME}/
)

ament_package()
