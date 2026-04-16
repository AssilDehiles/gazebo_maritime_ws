import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/assil/gazebo_maritime_ws/install/ros2_maritime'
