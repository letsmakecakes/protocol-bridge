import time
from pymavlink import mavutil

master = mavutil.mavlink_connection("192.168.0.102:14550", source_system=255, source_component=5)

master.wait_heartbeat()
print("Got heartbeat")


# Set mode to GUIDED

# Because of how ArduCopter works, need to look up a custom mode number
def get_custom_mode_number(mode):
    custom_mode_number = None
    # For ArduCopter, we use mode_mapping_acm, differs for ArduPlane or PX4
    # master.mode_mapping() should return the mapping needed
    for (number, mode_string) in mavutil.mode_mapping_acm.items():
        if mode_string == mode:
            custom_mode_number = number

    # Check if we actually found one
    if custom_mode_number is None:
        raise ValueError("Failed to find mode number for specified mode")

    return custom_mode_number


# Create set mode message
set_mode_message = mavutil.mavlink.MAVLink_command_long_message(
    1,  # Target system
    1,  # Target component
    mavutil.mavlink.MAV_CMD_DO_SET_MODE,  # Command
    0,  # Confirmation counter
    mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,  # Param 1 (Mode, for GUIDED, set to CUSTOM)
    get_custom_mode_number("GUIDED"),  # Param 2 (Custom mode, GUIDED = 4 for ArduCopter)
    0,  # Param 3 (Custom sub-mode, unused for ArduPilot)
    0,  # Param 4 (Unused)
    0,  # Param 5 (Unused)
    0,  # Param 6 (Unused)
    0  # Param 7 (Unused)
)

# Send set mode message
master.mav.send(set_mode_message)

# Check that our DO_SET_MODE command was successful
msg = master.recv_match(type="COMMAND_ACK", blocking=True)
print(msg)
if msg.result != mavutil.mavlink.MAV_RESULT_ACCEPTED:
    print("Error setting mode")
    exit()


# Define a function used to send commands in the future
# Unused params are left at 0
def send_command(command, confirmation, param1=0, param2=0, param3=0, param4=0, param5=0, param6=0, param7=0):
    """
    Send a COMMAND_LONG message to (sys,comp) = (1,1)
    """
    master.mav.command_long_send(
        1, 1,
        command,
        confirmation,
        param1,
        param2,
        param3,
        param4,
        param5,
        param6,
        param7
    )


# Now we need to arm the vehicle
print("Arming")
send_command(
    mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,  # Command
    0,  # Confirmation
    1,  # Param 1 (Arm)
    1,  # Param 2 (Force)
)

# Check if arming was successful
msg = master.recv_match(type="COMMAND_ACK", blocking=True)
print(msg)
if msg.result != mavutil.mavlink.MAV_RESULT_ACCEPTED:
    print("Error arming vehicle")
    exit()

# Command takeoff to 20m
send_command(
    mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,  # Command
    0,  # Confirmation
    param7=20  # Altitude (m)
)

# Check we sucessfully sent takeoff
msg = master.recv_match(type="COMMAND_ACK", blocking=True)
print(msg)
if msg.result != mavutil.mavlink.MAV_RESULT_ACCEPTED:
    print("Error sending takeoff command")
    exit()

# Wait for us to get to somewhere near 20m
msg = master.recv_match(type="GLOBAL_POSITION_INT", blocking=True)
while abs(20000 - msg.relative_alt) > 500:  # Alt in mm
    msg = master.recv_match(type="GLOBAL_POSITION_INT", blocking=True)

print("Takeoff complete")

# Get current system time from last msg
time_pair = (time.time(), msg.time_boot_ms)

# Setup the bitfields to tell the vehicle to ignore velocity and accelerations
ignore_velocity = (mavutil.mavlink.POSITION_TARGET_TYPEMASK_VX_IGNORE
                   | mavutil.mavlink.POSITION_TARGET_TYPEMASK_VY_IGNORE
                   | mavutil.mavlink.POSITION_TARGET_TYPEMASK_VZ_IGNORE
                   | mavutil.mavlink.POSITION_TARGET_TYPEMASK_YAW_RATE_IGNORE
                   )

ignore_accel = (mavutil.mavlink.POSITION_TARGET_TYPEMASK_AX_IGNORE
                | mavutil.mavlink.POSITION_TARGET_TYPEMASK_AY_IGNORE
                | mavutil.mavlink.POSITION_TARGET_TYPEMASK_AZ_IGNORE
                )

# Send a position target
master.mav.set_position_target_global_int_send(
    time_pair[1] + int(round((time.time() - time_pair[0]) * 1000)),
    1,
    1,
    mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT,
    (ignore_velocity | ignore_accel),
    int(51.4232303 * (10 ** 7)),  # Lat (degE7)
    int(-2.6710604 * (10 ** 7)),  # Long (degE7)
    100,  # Altitude
    0, 0, 0,  # Velocities
    0, 0, 0,  # Accels
    0,  # Yaw
    0  # Yaw rate
)

print("Set initial position")
time.sleep(0.5)
exit()


def set_position(lat, lng, alt):
    master.mav.set_position_target_global_int_send(
        time_pair[1] + int(round((time.time() - time_pair[0]) * 1000)),
        1,
        1,
        mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT,
        (ignore_velocity | ignore_accel),
        int(lat * (10 ** 7)),  # Lat (degE7)
        int(lng * (10 ** 7)),  # Long (degE7)
        alt,  # Altitude
        0, 0, 0,  # Velocities
        0, 0, 0,  # Accels
        0,  # Yaw
        0  # Yaw rate
    )


set_position(51.425, -2.671, 50)
