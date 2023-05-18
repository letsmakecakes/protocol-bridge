import asyncio
import json
import websockets
from pymavlink import mavutil


async def websocket_server(websocket, path):
    # Receive and process incoming messages
    async for message in websocket:
        # master = mavutil.mavlink_connection('192.168.0.104:14550')
        # master.wait_heartbeat()

        data = json.loads(message)
        #print("Received JSON data:", data)

        if data['arm'] == 1:
            # Send the response back to the client
            print('armed')
            # master.mav.command_long_send(master.target_system, master.target_component,
            #                              mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
            #                              0, 1, 0, 0, 0, 0, 0, 0)
            # while True:
            #     msg = master.recv_msg()
            #     if msg and msg.get_type() == 'COMMAND_ACK' and msg.command == mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM:
            #         if msg.result == mavutil.mavlink.MAV_RESULT_ACCEPTED:
            #             # send acknowledgement back to console
            #             break
            #         else:
            #             # send acknowledgement back to console
            #             # exit()
            #             break

        if data['disarm'] == 1:
            print("disarmed")
            # master.mav.command_long_send(
            #     master.target_system, master.target_component, mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
            #     0, 0,
            #     21196, 0,
            #     0, 0, 0, 0
            # )
            #
            # while True:
            #     msg = master.recv_msg()
            #     if msg and msg.get_type() == 'COMMAND_ACK' and msg.command == mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM:
            #         if msg.result == mavutil.mavlink.MAV_RESULT_ACCEPTED:
            #             # send acknowledgement back to console
            #             break
            #         else:
            #             # send acknowledgement back to console
            #             break

        # # Send control commands
        # master.mav.manual_control_send(
        # master.target_system,
        # roll, pitch, -throttle, yaw,
        # 0)  # set to 0 to indicate that buttons are not being used
        #
        # time.sleep(1)  # sleep for a short time to avoid overloading the serial port

        # Process the received data and create a response
        response = {"status": "drone received data", "message": "Received JSON data"}
        response_json = json.dumps(response)

        # Send the response back to the client
        await websocket.send(response_json)


start_server = websockets.serve(websocket_server, "localhost", 8765)
asyncio.get_event_loop().run_until_complete(start_server)
asyncio.get_event_loop().run_forever()
