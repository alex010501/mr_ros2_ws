source /opt/ros/humble/setup.bash

colcon build --symlink-install --packages-select mpc_controller

source install/setup.bash 
ros2 launch mpc_controller mpc_launch.py