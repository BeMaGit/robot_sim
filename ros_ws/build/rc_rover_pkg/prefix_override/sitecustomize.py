import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/user/Documents/robot_sim/ros_ws/install/rc_rover_pkg'
