# pico_go_project
This is a project to have between 3 and 5 automated picoGo, like AGVs

# 1. Clona
git clone https://github.com/mleo22d/pico_go_project.git
cd pico_go_project

# 2. (Re)compila el firmware si lo necesitas
cd micro_ros_raspberrypi_pico_sdk
mkdir -p build && cd build
cmake ..
make             # genera .uf2, .elf, etc. → quedan ignorados por Git
# luego flasheas tu PicoGo con picotool o tu script habitual

# 3. Compila el workspace ROS 2
cd ../../ros_ws        # vuelve al workspace
colcon build --symlink-install
source install/setup.bash   # o añade esto a tu ~/.bashrc

# 4. Lanza tu nodo o launch file
ros2 run motor_controller motor_controller          # una instancia
# o, cuando tengas launch:
ros2 launch pico_go_bringup single_pico.launch.py

