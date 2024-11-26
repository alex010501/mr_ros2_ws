# delete previous build
rm -r build install log
# source ros2
source /opt/ros/humble/setup.bash
# build depends
colcon build --symlink-install --packages-select Stage --cmake-args -DOpenGL_GL_PREFERENCE=LEGACY
colcon build --symlink-install --packages-select stage_ros2 angles stage_controller simple_controller simple_planner cart_launch vel_pub
# task 3
colcon build --symlink-install --packages-select mpc_controller
# task 5
colcon build --symlink-install --packages-select mapping
# task 7
colcon build --symlink-install --packages-select ekf_slam