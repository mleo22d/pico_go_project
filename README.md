# pico_go_project
This is a project to have between 3 and 5 automated picoGo, like AGVs

# 1. Clonw
git clone https://github.com/mleo22d/pico_go_project.git
cd pico_go_project

# 2. (Re)compile the firmware 
cd micro_ros_raspberrypi_pico_sdk
rm -rf build
mkdir -p build && cd build
cmake -DPICO_BOARD=pico_w ..
make -j4

# 3. Compile ROS2 workspace
cd ../../ros_ws       
colcon build --symlink-install
source install/setup.bash  


