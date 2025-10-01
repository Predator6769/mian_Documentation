cmake_minimum_required(VERSION 3.8)
project(gnss_imu_quick_convert)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(radar_msgs REQUIRED)
find_package(gps_msgs REQUIRED)
find_package(novatel_oem7_msgs REQUIRED)
find_package(autoware_sensing_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(angles REQUIRED)
find_package(std_msgs REQUIRED)
find_package(nav_msgs REQUIRED)


add_executable(gnss_imu_quick_convert_node
  src/gnss_imu_quick_convert_node.cpp
)
ament_target_dependencies(gnss_imu_quick_convert_node
  rclcpp
  sensor_msgs
  radar_msgs
  gps_msgs
  novatel_oem7_msgs
  autoware_sensing_msgs
  tf2
  tf2_geometry_msgs
  angles
  nav_msgs
  std_msgs
)

install(TARGETS gnss_imu_quick_convert_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}/
)

ament_package()
