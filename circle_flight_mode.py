"""
Example of how to change flight modes using pymavlink
"""

import sys
# Import mavutil
from pymavlink import mavutil

# Create the connection
master = mavutil.mavlink_connection('192.168.0.108:14550')
# Wait a heartbeat before sending commands
master.wait_heartbeat()

# Choose a mode
mode = 'CIRCLE'

# Check if mode is available
if mode not in master.mode_mapping():
    print('Unknown mode : {}'.format(mode))
    print('Try:', list(master.mode_mapping().keys()))
    sys.exit(1)

# Get mode ID
mode_id = master.mode_mapping()[mode]
print(mode_id)

# Set new mode
master.mav.set_mode_send(
    master.target_system,
    mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
    mode_id)

# Check ACK
ack = False
while not ack:
    ack_msg = master.recv_match(type='COMMAND_ACK', blocking=True)
    ack_msg = ack_msg.to_dict()
    print(ack_msg)
    if ack_msg['command'] != mavutil.mavlink.MAVLINK_MSG_ID_SET_MODE:
        continue
    ack = True
    if ack_msg['result'] != 0:
        print('Changing mode failed')
        sys.exit(1)
    print('Mode changed')