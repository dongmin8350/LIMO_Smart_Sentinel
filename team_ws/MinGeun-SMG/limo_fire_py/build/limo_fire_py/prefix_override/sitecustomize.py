import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/wego/team_ws/MinGeun-SMG/limo_fire_py-main/install/limo_fire_py'
