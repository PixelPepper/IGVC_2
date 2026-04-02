cmake_minimum_required(VERSION 3.8)
project(estop_ros)

set(CMAKE_CXX_STANDARD 17)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(serial REQUIRED)

include_directories(include)

add_executable(cmd_vel_override_node src/cmd_vel_override.cpp)
ament_target_dependencies(cmd_vel_override_node rclcpp std_msgs geometry_msgs serial)

install(TARGETS
  cmd_vel_override_node
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME})

ament_package()
