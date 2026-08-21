cmake_minimum_required(VERSION 3.8)
project(pointcloud_preprocessor)

if(POLICY CMP0074)
  cmake_policy(SET CMP0074 NEW)
endif()

if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(PCL REQUIRED COMPONENTS common filters sample_consensus segmentation)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_sensor_msgs REQUIRED)

add_executable(pointcloud_preprocessor_node
  src/pointcloud_preprocessor_node.cpp
)
target_compile_features(pointcloud_preprocessor_node PUBLIC cxx_std_17)
target_include_directories(pointcloud_preprocessor_node SYSTEM PRIVATE ${PCL_INCLUDE_DIRS})
ament_target_dependencies(pointcloud_preprocessor_node
  pcl_conversions
  rclcpp
  sensor_msgs
  std_msgs
  tf2
  tf2_ros
  tf2_sensor_msgs
)
target_link_libraries(pointcloud_preprocessor_node ${PCL_LIBRARIES})

install(TARGETS pointcloud_preprocessor_node
  DESTINATION lib/${PROJECT_NAME}
)
install(DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
