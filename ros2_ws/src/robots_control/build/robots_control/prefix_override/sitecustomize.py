import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/user/pico_go_project/ros2_ws/src/robots_control/install/robots_control'
