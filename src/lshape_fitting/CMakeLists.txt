cmake_minimum_required(VERSION 3.16)
project(lshape_fitting LANGUAGES C CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

# --- msg generation dependencies ---
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)

# ----------------------------
# Generate custom interfaces
# ----------------------------
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/Detection2D.msg"
  "msg/Detection2DArray.msg"
  DEPENDENCIES std_msgs geometry_msgs
)

# ----------------------------
# Node target
# ----------------------------
add_executable(lshape_fitting_node src/lshape_fitting_node.cpp)
ament_target_dependencies(
  lshape_fitting_node
  rclcpp
  sensor_msgs
  geometry_msgs
  visualization_msgs
  tf2
  tf2_geometry_msgs
)

rosidl_target_interfaces(lshape_fitting_node
  ${PROJECT_NAME} "rosidl_typesupport_cpp")

install(TARGETS lshape_fitting_node
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}/
)

install(DIRECTORY msg
  DESTINATION share/${PROJECT_NAME}/
)

ament_export_dependencies(rosidl_default_runtime)
ament_package()
