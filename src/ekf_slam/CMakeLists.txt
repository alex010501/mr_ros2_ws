cmake_minimum_required(VERSION 3.5)
project(ekf_slam)

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
    set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(angles REQUIRED)
find_package(Eigen3 REQUIRED)

# include directories
include_directories(include)
include_directories(${EIGEN3_INCLUDE_DIR})

# Add source files
add_executable(ekf_slam src/main.cpp src/slam.cpp)
ament_target_dependencies(ekf_slam
    rclcpp
    std_msgs
    sensor_msgs
    nav_msgs    
    geometry_msgs
    visualization_msgs
    tf2
    tf2_ros
    tf2_geometry_msgs
    angles
    Eigen3
)

# Install targets
install(TARGETS ekf_slam
    RUNTIME DESTINATION lib/${PROJECT_NAME}
    LIBRARY DESTINATION lib
    ARCHIVE DESTINATION lib
)

# Install headers
install(DIRECTORY include/
    DESTINATION include
)

# Install launch files
install(DIRECTORY launch
    DESTINATION share/${PROJECT_NAME}
)

# Ament package
ament_package()
