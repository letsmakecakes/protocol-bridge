import time
from pymavlink import mavutil

# Connect to the Cube flight controller
master = mavutil.mavlink_connection('COM5', baud=9600)

# Wait for the heartbeat message to start receiving data
master.wait_heartbeat()

# Main loop
while True:
    try:
        # Check system status
        print("System status: ", mavutil.mavlink.enums['MAV_STATE'][master.messages['HEARTBEAT'].system_status].name)

        # Check battery voltage
        print("Battery voltage: ", master.messages['SYS_STATUS'].voltage_battery / 1000.0, "V")

        # Check GPS fix status
        gps_fix = master.messages['GPS_RAW_INT'].fix_type
        if gps_fix == 3:
            print("GPS fix: 3D Fix")
        elif gps_fix == 2:
            print("GPS fix: 2D Fix")
        else:
            print("GPS fix: No fix")

        # Check vehicle mode
        print("Vehicle mode: ", mavutil.mavlink.enums['MAV_MODE_FLAG'][master.messages['SYS_STATUS'].mode].name)

        time.sleep(1)  # Delay between checks

    except KeyboardInterrupt:
        break

# Close the connection
master.close()
