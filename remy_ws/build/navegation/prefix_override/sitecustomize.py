import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/chocolate/SEMEAR/remy/remy_ws/install/navegation'
