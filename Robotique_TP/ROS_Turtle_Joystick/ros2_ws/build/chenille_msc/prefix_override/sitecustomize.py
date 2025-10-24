import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/ensea/Documents/ROS_Turtle_Joystick/ros2_ws/install/chenille_msc'
