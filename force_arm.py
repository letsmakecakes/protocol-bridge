import time
from pymavlink import mavutil

def set_waypoint(conn, latitude, longitude, altitude, seq):
    msg = conn.mav.mission_item_encode(
        0, 0, seq, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
        mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 2, 1, 0, 0, 0, latitude, longitude, altitude)
    conn.send(msg)

    # Wait for acknowledgment message
    ack_msg = conn.recv_match(type='MISSION_ITEM_REACHED', blocking=True)
    if ack_msg:
        print(f"Waypoint {seq} reached.")

def main():
    # Connect to SITL
    sitl = None
    try:
        sitl = mavutil.mavlink_connection('192.168.0.102:14550')
    except Exception as e:
        print('Error connecting to SITL:', str(e))
        return

    # Wait for the connection to establish
    sitl.wait_heartbeat()

    # Clear existing mission
    sitl.mav.mission_clear_all_send(target_system=1, target_component=1)
    sitl.mav.mission_count_send(0)

    # Set waypoints
    waypoints = [
        (47.398039859999997, 8.5455725400000002, 10),  # Waypoint 1 (latitude, longitude, altitude)
        (47.398036222362471, 8.5450146439425509, 20),  # Waypoint 2
        (47.397825620791885, 8.5450092830163271, 30),  # Waypoint 3
    ]
    seq = 0
    for waypoint in waypoints:
        set_waypoint(sitl, *waypoint, seq)
        seq += 1
        time.sleep(1)  # Delay between setting waypoints

    # Send mission count to confirm waypoints
    sitl.mav.mission_count_send(seq)

    # Disconnect from SITL
    sitl.close()

if __name__ == '__main__':
    main()
