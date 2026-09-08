cmake_minimum_required(VERSION 3.8)
project(obstacle_filtering_real)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -O3)
endif()

find_package(ament_cmake REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)

add_executable(obstacle_filtering_node
  src/obstacle_filtering_node.cpp
  src/waypoint_graph.cpp
)

target_include_directories(obstacle_filtering_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(obstacle_filtering_node
  ament_index_cpp
  rclcpp
  sensor_msgs
  nav_msgs
  tf2
  tf2_ros
  geometry_msgs
  std_msgs
)

target_compile_features(obstacle_filtering_node PUBLIC cxx_std_17)

install(TARGETS obstacle_filtering_node
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY include/
  DESTINATION include)

install(DIRECTORY launch config matlab
  DESTINATION share/${PROJECT_NAME})

ament_package()
