cmake_minimum_required(VERSION 3.8)
project(stage_controller)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

add_executable(stage_controller src/stage_controller.cpp)
add_executable(stage_throttle src/stage_throttle.cpp)

ament_target_dependencies(
  stage_controller
  rclcpp
  std_msgs
  geometry_msgs
)

ament_target_dependencies(
  stage_throttle
  rclcpp
  std_msgs
  geometry_msgs
)

install(TARGETS
  stage_controller
  stage_throttle
  DESTINATION lib/${PROJECT_NAME})

ament_package()