cmake_minimum_required(VERSION 3.5)
project(map_comparator)

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(OpenCV REQUIRED)

include_directories(include)

# Add executables for each node
add_executable(show_current_map src/show_current_map.cpp)
add_executable(show_finished_map src/show_finished_map.cpp)

# Link OpenCV
target_link_libraries(show_current_map ${OpenCV_LIBRARIES})
target_link_libraries(show_finished_map ${OpenCV_LIBRARIES})

# Find dependencies for each node
ament_target_dependencies(show_current_map rclcpp nav_msgs OpenCV)
ament_target_dependencies(show_finished_map rclcpp nav_msgs OpenCV)

# Install the executables
install(TARGETS
  show_current_map
  show_finished_map
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
