cmake_minimum_required(VERSION 3.5)
project(sprint_2_localiser)

# Dependencies
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(OpenCV REQUIRED)
find_package(yaml-cpp REQUIRED)

# Include directories
include_directories(
  include
  ${OpenCV_INCLUDE_DIRS}
)

# Executable
add_executable(scan_matching_localiser src/scan_matching_localiser.cpp)

# Link libraries
target_link_libraries(scan_matching_localiser
  ${OpenCV_LIBRARIES}
  yaml-cpp  # Link yaml-cpp explicitly
)

# Specify package dependencies
ament_target_dependencies(scan_matching_localiser
  rclcpp
  sensor_msgs
  geometry_msgs
  nav_msgs
  yaml-cpp
)

install(TARGETS
  scan_matching_localiser
  DESTINATION lib/${PROJECT_NAME})

ament_package()
