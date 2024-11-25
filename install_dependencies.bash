# Install ACADO toolkit
sudo apt update && sudo apt upgrade -y
sudo apt install git cmake g++ libjpeg8-dev libpng-dev libglu1-mesa-dev libltdl-dev libfltk1.1-dev libeigen3-dev
cd ACADOtoolkit
mkdir build
cd build
cmake ..
make
sudo make install
# Build other dependencies
cd ../..
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-select Stage --cmake-args -DOpenGL_GL_PREFERENCE=LEGACY
colcon build --symlink-install --packages-select stage_ros2 angles stage_controller simple_controller simple_planner cart_launch