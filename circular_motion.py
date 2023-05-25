import math
import sys
import time
from pymavlink import mavutil

# Set up the connection to the autopilot
# connection_string = '/dev/ttyACM0'  # Update with your connection string
master = mavutil.mavlink_connection('192.168.0.108:14550')

# Wait for the heartbeat message to establish a connection
master.wait_heartbeat()

# Set mode to guided
mode = 'GUIDED'
custom_mode = 4  # Custom mode for guided, check your autopilot's documentation
mode_mapping = master.mode_mapping()
mode_id = mode_mapping[mode]
master.mav.set_mode_send(
    master.target_system,
    mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
    custom_mode
)

# Acknowledge command
ack = False
while not ack:
    ack_msg = master.recv_match(type='COMMAND_ACK', blocking=True).to_dict()
    if ack_msg['command'] != mavutil.mavlink.MAVLINK_MSG_ID_SET_MODE:
        continue
    ack = True
    if ack_msg['result'] != 0:
        print('Changing mode failed')
        sys.exit(1)
    print('Mode changed')

while True:
    current_mode = master.mode_mapping()[mode]
    if current_mode == mode_id:
        break
    time.sleep(1)

# Arm the vehicle
master.arducopter_arm()

# Takeoff to a desired altitude
target_altitude = 10.0  # meters
master.mav.command_long_send(
    master.target_system,
    master.target_component,
    mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,
    0,
    0, 0, 0, 0, 0, 0, target_altitude
)


# Wait until the vehicle reaches the target altitude
while True:
    msg = master.recv_match(type='GLOBAL_POSITION_INT', blocking=True)
    altitude = msg.alt / 1000.0  # Convert to meters
    if altitude >= target_altitude * 0.95:
        break
    time.sleep(1)

# Fly in circles using SET_POSITION_TARGET_LOCAL_NED command
radius = 10.0  # meters
center_lat = msg.lat * 1e-7
center_lon = msg.lon * 1e-7
waypoint_count = 20
angle_increment = 2 * 3.14159 / waypoint_count

for i in range(waypoint_count):
    angle = i * angle_increment
    north = center_lat + radius * math.cos(angle)
    east = center_lon + radius * math.sin(angle)
    target_altitude = 10.0  # meters
    yaw = 0.0  # degrees

    msg = master.mav.set_position_target_local_ned_encode(
        0,  # Time boot ms
        master.target_system,
        master.target_component,
        mavutil.mavlink.MAV_FRAME_LOCAL_NED,
        0b0000111111111000,  # Enable position, velocity, and yaw
        north,
        east,
        -target_altitude,
        0, 0, 0,  # Velocity components (not used)
        0, 0, 0,  # Acceleration components (not used)
        yaw * 100  # Yaw angle (degrees) * 100
    )

    master.mav.send(msg)
    time.sleep(1)  # Wait for 1 second between each waypoint

# Land the vehicle
master.mav.command_long_send(
    master.target_system,
    master.target_component,
    mavutil.mavlink.MAV_CMD_NAV_LAND,
    0,
    0, 0, 0, 0, 0, 0, 0
)

# Wait until the vehicle has landed
while True:
    msg = master.recv_match(type='GLOBAL_POSITION_INT', blocking=True)
    altitude = msg.alt / 1000.0  # Convert to meters
    if altitude <= 0.1:
        break
    time.sleep(1)

# Disarm the vehicle
master.arducopter_disarm()

# Close the connection
master.close()
