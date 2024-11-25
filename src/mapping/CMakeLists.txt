cmake_minimum_required(VERSION 3.8)
project(mapping)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()
# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)

# Add source files
add_executable(mapping src/simple_map.cpp)
ament_target_dependencies(mapping
    rclcpp
    nav_msgs
    std_msgs
    sensor_msgs
    geometry_msgs
    tf2
    tf2_ros
)

# Install targets
install(TARGETS mapping
    RUNTIME DESTINATION lib/${PROJECT_NAME}
    LIBRARY DESTINATION lib
    ARCHIVE DESTINATION lib
)

# Install launch files
install(DIRECTORY launch
    DESTINATION share/${PROJECT_NAME}
)

# Install RQT and RViz configurations
install(DIRECTORY rviz
    DESTINATION share/${PROJECT_NAME}
)

# Ament package
ament_package()
