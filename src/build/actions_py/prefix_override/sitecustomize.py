import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/chinmay/ROS2_Humble-Actions/src/install/actions_py'
