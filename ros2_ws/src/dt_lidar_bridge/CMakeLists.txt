cmake_minimum_required(VERSION 3.16)
project(dt_lidar_bridge)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()
add_compile_options(-Wall -Wextra -Wpedantic)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(livox_ros_driver2 REQUIRED)

# Pure conversion logic, no ROS node machinery, so it can be unit tested.
add_library(dt_canonical STATIC src/canonical.cpp)
target_include_directories(dt_canonical PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
set_target_properties(dt_canonical PROPERTIES POSITION_INDEPENDENT_CODE ON)
ament_target_dependencies(dt_canonical sensor_msgs)

add_library(dt_lidar_bridge_components SHARED
  src/points_bridge.cpp
  src/imu_bridge.cpp)
target_link_libraries(dt_lidar_bridge_components dt_canonical)
ament_target_dependencies(dt_lidar_bridge_components
  rclcpp rclcpp_components sensor_msgs livox_ros_driver2)

rclcpp_components_register_node(dt_lidar_bridge_components
  PLUGIN "dt_lidar_bridge::PointsBridge"
  EXECUTABLE points_bridge)
rclcpp_components_register_node(dt_lidar_bridge_components
  PLUGIN "dt_lidar_bridge::ImuBridge"
  EXECUTABLE imu_bridge)

install(TARGETS dt_canonical dt_lidar_bridge_components
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)
install(DIRECTORY include/ DESTINATION include)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_canonical test/test_canonical.cpp)
  target_link_libraries(test_canonical dt_canonical)
endif()

ament_package()
