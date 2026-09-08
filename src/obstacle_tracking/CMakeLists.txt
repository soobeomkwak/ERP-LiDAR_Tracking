cmake_minimum_required(VERSION 3.16)
project(obstacle_tracking LANGUAGES C CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(lshape_fitting REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/TrackDebug.msg"
  "msg/TrackDebugArray.msg"
  "msg/Track2D.msg"
  "msg/Track2DArray.msg"
  DEPENDENCIES std_msgs geometry_msgs
)

add_executable(obstacle_tracking_node src/obstacle_tracking_node.cpp)
ament_target_dependencies(
  obstacle_tracking_node
  rclcpp
  std_msgs
  geometry_msgs
  nav_msgs
  sensor_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
  visualization_msgs
  lshape_fitting
)

rosidl_target_interfaces(obstacle_tracking_node
  ${PROJECT_NAME} "rosidl_typesupport_cpp")

install(TARGETS obstacle_tracking_node
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(PROGRAMS
  scripts/analyze_tracking_bag.py
  scripts/analyze_tracked_state_bag.py
  scripts/check_obstacle_absolute_state.py
  scripts/check_relative_velocity_consistency.py
  scripts/check_realtime_sensor_alignment.py
  scripts/diagnose_tracking_velocity_flow.py
  scripts/check_stamp_alignment.py
  scripts/validate_perception_outputs.py
  scripts/monitor_perception_pipeline.py
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}/
)

install(DIRECTORY msg
  DESTINATION share/${PROJECT_NAME}/
)

ament_export_dependencies(rosidl_default_runtime)
ament_package()
