import asyncio
import json
import sys
import websockets
from pymavlink import mavutil


async def websocket_server(websocket, path):
    # Receive and process incoming messages
    async for message in websocket:
        data = json.loads(message)
        print("Received JSON data:", data)

        # Process the received data and create a response
        response = {"status": "drone received data", "message": "Received JSON data"}
        response_json = json.dumps(response)

        # Send the response back to the client
        await websocket.send(response_json)

        # Arming the flight controller
        if data['arm'] == 1:
            master.mav.command_long_send(
                master.target_system,
                master.target_component,
                mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
                0, 1, 0, 0, 0, 0, 0, 0
            )
            while True:
                msg = master.recv_msg()
                if msg and msg.get_type() == 'COMMAND_ACK' and msg.command == mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM:
                    if msg.result == mavutil.mavlink.MAV_RESULT_ACCEPTED:
                        print("Flight controller armed")
                        break
                    else:
                        print('Error arming the flight controller')
                        break

        # Disarming the flight controller
        if data['disarm'] == 1:
            master.mav.command_long_send(
                master.target_system,
                master.target_component,
                mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
                0, 0, 21196, 0, 0, 0, 0, 0
            )

            while True:
                msg = master.recv_msg()
                if msg and msg.get_type() == 'COMMAND_ACK' and msg.command == mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM:
                    if msg.result == mavutil.mavlink.MAV_RESULT_ACCEPTED:
                        print("Flight controller disarmed")
                        break
                    else:
                        print("Error disarming the flight controller")
                        break

        # Setting the flight controller mode to LOITER
        if data['mode'] == 1:
            my_dict = {'STABILIZE': 0, 'ACRO': 1, 'ALT_HOLD': 2, 'AUTO': 3, 'GUIDED': 4, 'LOITER': 5, 'RTL': 6, 'CIRCLE': 7, 'POSITION': 8, 'LAND': 9, 'OF_LOITER': 10, 'DRIFT': 11, 'SPORT': 13, 'FLIP': 14, 'AUTOTUNE': 15, 'POSHOLD': 16, 'BRAKE': 17, 'THROW': 18, 'AVOID_ADSB': 19, 'GUIDED_NOGPS': 20, 'SMART_RTL': 21, 'FLOWHOLD': 22, 'FOLLOW': 23, 'ZIGZAG': 24, 'SYSTEMID': 25, 'AUTOROTATE': 26, 'AUTO_RTL': 27}
            mode = 'LOITER'
            print(f"Setting the drone in {mode} mode")
            if mode not in my_dict:
                print('Unknown mode:', mode)
                print('Try:', list(my_dict.keys()))
                sys.exit(1)
            mode_id = my_dict[mode]
            master.set_mode(mode_id)
            while True:
                ack_msg = master.recv_match(type='COMMAND_ACK', blocking=True)
                ack_msg = ack_msg.to_dict()
                if ack_msg['command'] != mavutil.mavlink.MAV_CMD_DO_SET_MODE:
                    continue
                print(mavutil.mavlink.enums['MAV_RESULT'][ack_msg['result']].description)
                break

        try:
            # Send control commands to the flight controller
            master.mav.manual_control_send(
                master.target_system,
                data['roll'],
                data['pitch'],
                -data['throttle'],
                data['yaw'],
                0  # set to 0 to indicate that buttons are not being used
            )

            # Sleep for a short time to avoid overloading the serial port
            await asyncio.sleep(0.01)
        except:
            continue


if __name__ == '__main__':
    master = mavutil.mavlink_connection('192.168.0.105:14550')
    master.wait_heartbeat()

    start_server = websockets.serve(websocket_server, "localhost", 8765)
    asyncio.get_event_loop().run_until_complete(start_server)
    asyncio.get_event_loop().run_forever()
