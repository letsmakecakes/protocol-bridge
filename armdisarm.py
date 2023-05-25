"""
Example of how to Arm and Disarm an Autopilot with pymavlink
"""
# Import mavutil
from pymavlink import mavutil

# Create the connection
master = mavutil.mavlink_connection('192.168.0.108:14550')
# Wait a heartbeat before sending commands
master.wait_heartbeat()

# https://mavlink.io/en/messages/common.html#MAV_CMD_COMPONENT_ARM_DISARM

# Arm
# master.arducopter_arm() or:
master.mav.command_long_send(
    master.target_system,
    master.target_component,
    mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
    0,
    1, 0, 0, 0, 0, 0, 0)

# wait until arming confirmed (can manually check with master.motors_armed())
print("Waiting for the vehicle to arm")
master.arducopter_arm()
#master.motors_armed_wait()
print('Armed!')

# Disarm
# master.arducopter_disarm() or:
master.mav.command_long_send(
    master.target_system,
    master.target_component,
    mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
    0,
    0, 0, 0, 0, 0, 0, 0)

# wait until disarming confirmed
master.motors_disarmed_wait()
print('Disarmed!')