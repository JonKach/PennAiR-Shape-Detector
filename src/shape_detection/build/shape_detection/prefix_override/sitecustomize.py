import sys
if sys.prefix == '/Users/jonkach/miniconda3/envs/ros2':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/Users/jonkach/Documents/Projects/VSCodeProjects/ros2_ws/src/shape_detection/install/shape_detection'
