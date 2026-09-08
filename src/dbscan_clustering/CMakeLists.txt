cmake_minimum_required(VERSION 3.16)
project(dbscan_clustering LANGUAGES CXX CUDA)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Configure CUDA language standard and architectures to avoid CMake errors/warnings
if (COMMAND cmake_policy)
  cmake_policy(SET CMP0104 NEW)
endif()
if(NOT DEFINED CMAKE_CUDA_ARCHITECTURES)
  # CUDA 13 no longer accepts older targets such as compute_60.
  # Keep the default on currently supported NVIDIA GPU generations.
  # Adjust or override in the top-level CMake invocation if needed.
  set(CMAKE_CUDA_ARCHITECTURES 75 80 86 89 90 CACHE STRING "CUDA architectures" FORCE)
endif()
set(CMAKE_CUDA_STANDARD 14)
set(CMAKE_CUDA_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
include_directories(include)

add_library(dbscan_gpu SHARED src/dbscan_gpu.cu)
target_include_directories(dbscan_gpu PRIVATE include)
set_target_properties(dbscan_gpu PROPERTIES POSITION_INDEPENDENT_CODE ON)
# Force a sane CUDA architecture list on the target to avoid 'compute_native' being used by nvcc
set_target_properties(dbscan_gpu PROPERTIES CUDA_ARCHITECTURES "${CMAKE_CUDA_ARCHITECTURES}")

add_executable(dbscan_clustering_node src/dbscan_clustering_node.cpp)
target_include_directories(dbscan_clustering_node PRIVATE include)
target_link_libraries(dbscan_clustering_node dbscan_gpu)
set_target_properties(dbscan_clustering_node PROPERTIES
  BUILD_RPATH "$ORIGIN"
  INSTALL_RPATH "$ORIGIN"
)

ament_target_dependencies(dbscan_clustering_node rclcpp sensor_msgs std_msgs)

install(TARGETS dbscan_clustering_node dbscan_gpu
  RUNTIME DESTINATION lib/${PROJECT_NAME}
  LIBRARY DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}/
)

ament_package()
