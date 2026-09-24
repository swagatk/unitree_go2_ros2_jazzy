import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/kumars/unitree_go2_ros2_jazzy/install/slam_scripts'
