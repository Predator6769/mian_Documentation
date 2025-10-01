cmake_minimum_required(VERSION 3.8)
project(velodyne_quick_converter)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)

add_executable(velodyne_quick_converter_node
  src/velodyne_quick_converter_node.cpp
)
ament_target_dependencies(velodyne_quick_converter_node
  rclcpp
  sensor_msgs
)

install(TARGETS velodyne_quick_converter_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}/
)

ament_package()


