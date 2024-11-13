cmake_minimum_required(VERSION 3.5)
project(mpc_controller)

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
find_package(geometry_msgs REQUIRED)

find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)

find_package(ACADO REQUIRED)

find_package(Boost REQUIRED COMPONENTS system)
find_package(Eigen3 REQUIRED)

include_directories(
    ${ACADO_INCLUDE_DIRS}
    include
)
link_directories(${ACADO_LIBRARY_DIRS})

add_executable(mpc_controller src/mpccontroller_node.cpp
                              src/mpccontroller.cpp
                              src/mpc.cpp)

ament_target_dependencies(mpc_controller rclcpp std_msgs sensor_msgs geometry_msgs tf2 tf2_ros)

target_link_libraries(mpc_controller
    ${ACADO_SHARED_LIBRARIES}
)

add_executable(mpc_test src/mpc.cpp test/mpc_test.cpp)

ament_target_dependencies(mpc_test rclcpp std_msgs sensor_msgs geometry_msgs tf2 tf2_ros)

target_link_libraries(mpc_test
    ${ACADO_SHARED_LIBRARIES}
)