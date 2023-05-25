import math

from pymavlink import mavutil


# Class for forwarding mission items
class MissionItem:
    def __init__(self, seq, current, x, y, z):
        self.seq = seq
        self.frame = mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT  # Use global altitude and longitude for positioning
        self.command = mavutil.mavlink.MAV_CMD_NAV_WAYPOINT  # Use the MAV_CMD_NAV_WAYPOINT command to set a waypoint
        self.current = current
        self.auto = 1
        self.param1 = 0.0
        self.param2 = 2.00
        self.param3 = 20.00
        self.param4 = math.nan
        self.param5 = x
        self.param6 = y
        self.param7 = z
        self.mission_type = 0  # The MAV_MISSION_TYPE value for MAV_MISSION_TYPE_MISSION


# Arm the drone
def arm(the_connection):
    print('Arming')
    the_connection.mav.command_long_send(the_connection.target_system, the_connection.target_component,
                                         mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 0, 1, 0, 0, 0, 0, 0, 0)

    # the_connection.quadcopter_arm()

    ack(the_connection, "COMMAND_ACK")


# Takeoff the drone
def takeoff(the_connection):
    print('Taking Off')
    the_connection.mav.command_long_send(the_connection.target_system, the_connection.target_component,
                                         mavutil.mavlink.MAV_CMD_NAV_TAKEOFF, 0, 0, 0, 0, math.nan, 0, 0, 10)

    ack(the_connection, "COMMAND_ACK")


# Upload the mission items to the drone
def upload_mission(the_connection, mission_items):
    n = len(mission_items)
    print('Sending message out')
    the_connection.mav.mission_count_send(the_connection.target_system, the_connection.target_component, n, 0)

    ack(the_connection, "MISSION_REQUEST")

    for waypoint in mission_items:
        print('Creating a waypoint')
        the_connection.mav.mission_item_send(the_connection.target_system,  # target_system
                                             the_connection.target_component,  # target_component
                                             waypoint.seq,  # sequence
                                             waypoint.frame,  # frame
                                             waypoint.command,  # command
                                             waypoint.current,  # current
                                             waypoint.auto,  # auto continue
                                             waypoint.param1,  # Hold Time
                                             waypoint.param2,  # Acceptance Radius
                                             waypoint.param3,  # Pass Radius
                                             waypoint.param4,  # Yaw
                                             waypoint.param5,  # Latitude
                                             waypoint.param6,  # Longitude
                                             waypoint.param7,  # Altitude
                                             waypoint.mission_type  # Mission Type
                                             )
        if waypoint != mission_items[n - 1]:
            ack(the_connection, "MISSION_REQUEST")

    ack(the_connection, "MISSION_ACK")


# Send message for the drone to return to the launch point
def set_return(the_connection):
    print('Set Return To Launch')
    the_connection.mav.command_long_send(the_connection.target_system, the_connection.target_component,
                                         mavutil.mavlink.MAV_CMD_NAV_RETURN_TO_LAUNCH, 0, 0, 0, 0, 0, 0, 0, 0)

    ack(the_connection, "COMMAND_ACK")


# Start mission
def start_mission(the_connection):
    print("Mission Start")
    the_connection.mav.command_long_send(the_connection.target_system, the_connection.target_component,
                                         mavutil.mavlink.MAV_CMD_MISSION_START, 0, 0, 0, 0, 0, 0, 0, 0)

    ack(the_connection, "COMMAND_ACK")


# Acknowledgement from the Drone
def ack(the_connection, keyword):
    print("Message Read " + str(the_connection.recv_match(type=keyword, blocking=True)))


# Main Function
if __name__ == '__main__':
    print('Program Started')
    the_connection = mavutil.mavlink_connection('192.168.0.108:14550')

    while the_connection.target_system == 0:
        print("Checking Heartbeat")
        the_connection.wait_heartbeat()
        print("Heartbeat from system (system %u component %u)" % (
            the_connection.target_system, the_connection.target_component))

    mission_waypoints = [MissionItem(0, 0, 42.434193622721835, -83.98698183753619, 10),
                         MissionItem(1, 0, 42.4343272622721835, -83.98698183753619, 10),
                         MissionItem(2, 0, 42.4343272622721835, -83.98698183753619, 10)]

    upload_mission(the_connection, mission_waypoints)

    arm(the_connection)

    takeoff(the_connection)

    start_mission(the_connection)

    for mission_item in mission_waypoints:
        print("Message read" + str(
            the_connection.recv_match(type='MISSION_ITEM_REACHED', condition='seq==' + str(mission_item.seq),
                                      blocking=True)))

    set_return(the_connection)
