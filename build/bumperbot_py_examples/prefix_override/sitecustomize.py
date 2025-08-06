import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/neel/bumperbot_ws2/install/bumperbot_py_examples'
