cmake_minimum_required(VERSION 3.8)
project(simple_controller)

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
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(angles REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

include_directories(include)

add_executable(simple_controller src/controller.cpp src/main.cpp)

ament_target_dependencies(
    simple_controller
    rclcpp
    std_msgs
    geometry_msgs
    sensor_msgs
    nav_msgs
    angles
    tf2
    tf2_ros
    tf2_geometry_msgs
)

install(TARGETS
    simple_controller
    DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY
    launch
    DESTINATION share/${PROJECT_NAME}/
)

# Install RQT and RViz configurations
install(DIRECTORY
    rqt rviz
    DESTINATION share/${PROJECT_NAME}
)

ament_package()
