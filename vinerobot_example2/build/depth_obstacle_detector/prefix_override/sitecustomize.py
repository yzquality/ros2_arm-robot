import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/yzquality/vinerobot_example/install/depth_obstacle_detector'
