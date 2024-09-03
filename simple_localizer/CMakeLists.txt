cmake_minimum_required(VERSION 3.5)
project(simple_localizer)

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)

add_executable(scan_matching_localiser src/scan_matching_localiser.cpp)

ament_target_dependencies(scan_matching_localiser rclcpp sensor_msgs)

install(TARGETS
scan_matching_localiser
  DESTINATION lib/${PROJECT_NAME})

ament_package()
