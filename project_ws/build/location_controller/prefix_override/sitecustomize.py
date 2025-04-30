import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/pablo-pc/GITHUB/TFG/project_ws/install/location_controller'
