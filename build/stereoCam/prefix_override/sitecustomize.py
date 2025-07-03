import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/deeric/belajar-ROS/ros2_ws/install/stereoCam'
