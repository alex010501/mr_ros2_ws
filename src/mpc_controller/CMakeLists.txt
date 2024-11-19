cmake_minimum_required(VERSION 3.8)
project(mpc_controller)

# Find dependencies
find_package(ACADO REQUIRED)
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(angles REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(Eigen3 REQUIRED)

# Find ACADO
# include_directories(/usr/local/include)
# link_directories(/usr/local/lib)

# Add source files
add_executable(mpc_controller src/main.cpp
                              src/mpccontroller.cpp
                              src/mpc.cpp)
target_include_directories(mpc_controller PUBLIC include ${ACADO_INCLUDE_DIRS})
ament_target_dependencies(mpc_controller
    rclcpp
    nav_msgs
    std_msgs
    sensor_msgs
    angles
    tf2
    tf2_geometry_msgs
    Eigen3
)
target_link_libraries(mpc_controller ${ACADO_SHARED_LIBRARIES})

# Install targets
install(TARGETS mpc_controller
    RUNTIME DESTINATION lib/${PROJECT_NAME}
    LIBRARY DESTINATION lib
    ARCHIVE DESTINATION lib
)

# Install headers
install(DIRECTORY include/
    DESTINATION include
)

# Install configuration files
install(DIRECTORY config
    DESTINATION share/${PROJECT_NAME}
)

# Install launch files
install(DIRECTORY launch
    DESTINATION share/${PROJECT_NAME}
)

# Install RQT and RViz configurations
install(DIRECTORY rqt rviz
    DESTINATION share/${PROJECT_NAME}
)

# Ament package
ament_package()
