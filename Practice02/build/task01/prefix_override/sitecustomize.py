import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/workspace/robotics_course/ros2-DarkMechanikum/Practice02/install/task01'
