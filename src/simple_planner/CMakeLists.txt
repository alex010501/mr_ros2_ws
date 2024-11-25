cmake_minimum_required(VERSION 3.8)
project(simple_planner)

# Add support for C++14
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(sensor_msgs REQUIRED)

# Include directories
include_directories(include)

# Build executable
add_executable(simple_planner src/planner.cpp src/main.cpp)
ament_target_dependencies(simple_planner
  rclcpp
  std_msgs
  geometry_msgs
  nav_msgs
  tf2
  tf2_ros
  sensor_msgs
)

install(TARGETS simple_planner
  DESTINATION lib/${PROJECT_NAME}
)

# Install launch files
install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}/
)

ament_package()
