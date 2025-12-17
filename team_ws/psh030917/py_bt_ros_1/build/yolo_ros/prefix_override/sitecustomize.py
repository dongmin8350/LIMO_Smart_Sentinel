import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/wego/team_ws/psh030917/py_bt_ros_1/install/yolo_ros'
