cmake_minimum_required(VERSION 3.8)
project(dgl_ros_gpd)

if(POLICY CMP0074)
  cmake_policy(SET CMP0074 NEW)
endif()

if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(dgl_ros_interfaces REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(OpenCV REQUIRED COMPONENTS core)
# GPD public headers include its plot helper, which in turn includes PCL/VTK
# visualization headers even when all GPD plot flags are disabled.
find_package(PCL REQUIRED COMPONENTS common filters visualization)
find_package(pcl_conversions REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)

set(GPD_ROOT "" CACHE PATH "Root of an installed atenpas/gpd library")
find_path(GPD_INCLUDE_DIR
  NAMES gpd/grasp_detector.h
  HINTS "${GPD_ROOT}/include" /usr/local/include
)
find_library(GPD_LIBRARY
  NAMES gpd_grasp_detector gpd
  HINTS "${GPD_ROOT}/lib" "${GPD_ROOT}/lib64" /usr/local/lib
)

if(NOT GPD_INCLUDE_DIR OR NOT GPD_LIBRARY)
  message(FATAL_ERROR
    "GPD was not found. Install https://github.com/atenpas/gpd, then configure "
    "with -DGPD_ROOT=/path/to/gpd/install if it is not under /usr/local."
  )
endif()

add_executable(gpd_action_server src/gpd_action_server.cpp)
target_compile_features(gpd_action_server PUBLIC cxx_std_17)
target_include_directories(gpd_action_server SYSTEM PRIVATE
  ${GPD_INCLUDE_DIR}
  ${OpenCV_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
)
ament_target_dependencies(gpd_action_server
  dgl_ros_interfaces
  Eigen3
  geometry_msgs
  pcl_conversions
  rclcpp
  rclcpp_action
  sensor_msgs
  tf2
  tf2_ros
)
target_link_libraries(gpd_action_server
  ${GPD_LIBRARY}
  ${OpenCV_LIBRARIES}
  ${PCL_LIBRARIES}
)

install(TARGETS gpd_action_server
  DESTINATION lib/${PROJECT_NAME}
)
install(DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
