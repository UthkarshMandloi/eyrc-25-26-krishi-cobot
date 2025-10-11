import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/bhaveshk01/colcon_ws/src/install/my_camera_pkg'
