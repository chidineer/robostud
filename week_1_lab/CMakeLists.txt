cmake_minimum_required(VERSION 3.5)
project(week_1_lab)

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED)

include_directories(${OpenCV_INCLUDE_DIRS})

add_executable(camera_subscriber src/camera_subscriber.cpp)
ament_target_dependencies(camera_subscriber rclcpp sensor_msgs std_msgs cv_bridge OpenCV)

install(TARGETS
  camera_subscriber
  DESTINATION lib/${PROJECT_NAME})

ament_package()
