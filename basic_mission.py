from pymavlink import mavutil

# Create a MAVLink connection
master = mavutil.mavlink_connection('192.168.0.102:14550')

# Wait for the heartbeat message from the autopilot
master.wait_heartbeat()

# Set the target system and component IDs
target_system = 1  # Autopilot system ID
target_component = 1  # Autopilot component ID


# Define a function to send a waypoint to the flight controller
def send_waypoint(latitude, longitude, altitude):
    # Create a MAVLink waypoint message
    msg = master.mav.mission_item_encode(
        target_system, target_component,
        0,  # Sequence number (0 for new mission)
        mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
        mavutil.mavlink.MAV_CMD_NAV_WAYPOINT,
        2,  # Auto-continue to next waypoint
        0, 0, 0, 0, 0, 0,
        latitude, longitude, altitude  # Waypoint coordinates
    )

    # Send the waypoint message
    master.mav.send(msg)

    # Wait for an acknowledgment message
    ack_msg = master.recv_match(type='MISSION_ACK', blocking=True)

    # Print the acknowledgment status
    print(f"Waypoint sent: {ack_msg.type}")


# Prompt the user for waypoint coordinates
latitude = int(input("Enter latitude (in degrees): "))
longitude = int(input("Enter longitude (in degrees): "))
altitude = int(input("Enter altitude (in meters): "))

# Send the waypoint to the flight controller
send_waypoint(latitude, longitude, altitude)
