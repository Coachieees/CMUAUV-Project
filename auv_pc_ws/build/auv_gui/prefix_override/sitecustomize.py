import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/coach-s-pc/auv_pc_ws/install/auv_gui'
