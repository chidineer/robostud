cmake_minimum_required(VERSION 3.8)
project(goal_sender)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(rclcpp_action REQUIRED)

add_executable(goal_sender src/goal_sender.cpp)
ament_target_dependencies(goal_sender rclcpp geometry_msgs nav2_msgs rclcpp_action)

install(TARGETS
  goal_sender
  DESTINATION lib/${PROJECT_NAME})

ament_package()