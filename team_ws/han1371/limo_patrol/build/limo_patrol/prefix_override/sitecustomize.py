import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/wego/team_ws/han1371/limo_patrol/install/limo_patrol'
