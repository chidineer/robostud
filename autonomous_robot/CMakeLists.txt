cmake_minimum_required(VERSION 3.5)
project(autonomous_robot)

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()

# Compiler options
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -Wunused-parameter)
else()
  add_compile_options(-Wunused-parameter)
endif()

# Find packages
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(OpenCV REQUIRED)
find_package(gazebo_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED)
find_package(rosbag2_cpp REQUIRED)
find_package(ament_index_cpp REQUIRED)

# Libraries
add_library(cylinder_detector src/cylinder_detector.cpp)
ament_target_dependencies(cylinder_detector rclcpp sensor_msgs tf2 tf2_geometry_msgs)

add_library(colour_detector src/colour_detector.cpp)
ament_target_dependencies(colour_detector rclcpp sensor_msgs cv_bridge OpenCV)

add_library(tsp src/tsp.cpp)
ament_target_dependencies(tsp rclcpp geometry_msgs)

add_library(localizer_and_navigation_library src/localizer_and_navigation.cpp)
ament_target_dependencies(localizer_and_navigation_library rclcpp sensor_msgs nav_msgs geometry_msgs nav2_msgs std_srvs cv_bridge OpenCV tf2 tf2_geometry_msgs)

# Executables
add_executable(autonomous_mapping src/autonomous_mapping.cpp src/autonomous_mapping_main.cpp)
ament_target_dependencies(autonomous_mapping rclcpp sensor_msgs nav_msgs geometry_msgs nav2_msgs std_srvs OpenCV)

add_executable(localizer_and_navigation src/localizer_and_navigation.cpp src/localizer_and_navigation_main.cpp)
ament_target_dependencies(localizer_and_navigation rclcpp sensor_msgs nav_msgs geometry_msgs nav2_msgs std_srvs cv_bridge OpenCV)
target_link_libraries(localizer_and_navigation cylinder_detector colour_detector tsp)

# Install targets
install(TARGETS
  autonomous_mapping
  localizer_and_navigation
  cylinder_detector
  colour_detector
  tsp
  localizer_and_navigation_library
  DESTINATION lib/${PROJECT_NAME})

# Build testing
if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)

  # Localization test
  ament_add_gtest(LOCALIZATION test/localization_test.cpp)
  ament_target_dependencies(LOCALIZATION rclcpp geometry_msgs nav_msgs rosbag2_cpp ament_index_cpp)
  target_link_libraries(LOCALIZATION localizer_and_navigation_library)

  # Navigation test
  ament_add_gtest(NAVIGATION test/navigation_test.cpp)
  ament_target_dependencies(NAVIGATION rclcpp geometry_msgs nav_msgs rosbag2_cpp ament_index_cpp)
  target_link_libraries(NAVIGATION localizer_and_navigation_library)

  # Box detection test
  ament_add_gtest(CATALOGUING test/box_detection_test.cpp)
  ament_target_dependencies(CATALOGUING rclcpp sensor_msgs rosbag2_cpp ament_index_cpp)
  target_link_libraries(CATALOGUING colour_detector)

  # TSP test
  ament_add_gtest(TASKALLOCATION test/tsp_test.cpp)
  ament_target_dependencies(TASKALLOCATION rclcpp geometry_msgs rosbag2_cpp ament_index_cpp)
  target_link_libraries(TASKALLOCATION tsp)
endif()

# Install other files
# install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})
# install(DIRECTORY config DESTINATION share/${PROJECT_NAME})
install(FILES package.xml DESTINATION share/${PROJECT_NAME})
install(DIRECTORY ros_bags DESTINATION share/${PROJECT_NAME})

ament_package()
