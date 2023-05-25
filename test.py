from pymavlink import mavutil

# Set up the connection to the flight controller
master = mavutil.mavlink_connection('192.168.0.102:14550')

# Wait for the heartbeat message to confirm successful connection
master.wait_heartbeat()

# Arm the drone
master.arducopter_arm()


# Send a command to set a waypoint
def send_waypoint(lat, lon, alt, seq):
    # Create the waypoint message
    msg = master.mav.mission_item_encode(
        master.target_system, master.target_component,
        seq, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
        mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 0, 0, 0, 0, 0, 0, lat, lon, alt
    )

    # Send the waypoint message
    master.mav.send(msg)

    # Request acknowledgment message
    master.mav.command_ack_send(
        msg.target_system, msg.target_component,
        msg.command, mavutil.mavlink.MAV_RESULT_IN_PROGRESS
    )


# Example waypoints
waypoints = [
    (47.123456, -122.654321, 10),  # Waypoint 1 (latitude, longitude, altitude)
    (47.234567, -122.765432, 20),  # Waypoint 2
    (47.345678, -122.876543, 30)  # Waypoint 3
]

# Send waypoints to the flight controller
for i, waypoint in enumerate(waypoints):
    send_waypoint(waypoint[0], waypoint[1], waypoint[2], i)

# Receive acknowledgment messages
while True:
    msg = master.recv_match(type='COMMAND_ACK', blocking=True)
    if msg:
        if msg.result == mavutil.mavlink.MAV_RESULT_ACCEPTED:
            print(f"Waypoint {msg} accepted by the flight controller")
        else:
            print(f"Waypoint {msg} rejected by the flight controller")
