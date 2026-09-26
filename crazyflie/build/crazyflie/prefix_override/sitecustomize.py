import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/abhishek/eysip_hardware/src/crazyflie/install/crazyflie'
