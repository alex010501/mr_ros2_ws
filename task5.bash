source /opt/ros/humble/setup.bash

colcon build --symlink-install --packages-select mapping

source install/setup.bash 
ros2 launch mapping map.launch.py