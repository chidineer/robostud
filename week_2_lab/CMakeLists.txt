cmake_minimum_required(VERSION 3.5)
project(week_2_lab)

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)

# Add executable
add_executable(laser_scan_subscriber src/laser_scan_subscriber.cpp)

# Specify libraries to link executable targets against
ament_target_dependencies(laser_scan_subscriber rclcpp sensor_msgs)

# Install the executable
install(TARGETS laser_scan_subscriber
  DESTINATION lib/${PROJECT_NAME})

ament_package()
