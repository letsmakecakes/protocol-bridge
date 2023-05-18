import time
from pymavlink import mavutil
import sys
import _thread
import rel
import json
import websocket

master = mavutil.mavlink_connection('192.168.0.104:14550')
master.wait_heartbeat()


def on_message(ws, message):
    json_data = json.loads(message)

    roll = int(json_data['roll'] * 1000)
    pitch = int(json_data['pitch'] * 1000)
    yaw = int(json_data['yaw'] * 1000)
    throttle = int(json_data['throttle'] * 1000)

    # roll = 0 if roll <=300 and roll >=-300 else roll
    # pitch = 0 if pitch <=300 and pitch >=-300 else pitch
    # yaw = 0 if yaw <=300 and yaw >=-300 else yaw
    # throttle = 0 if throttle <=300 and throttle >=-300 else throttle

    # if mode is in loiter
    if int(json_data['mode']) == 1:
        mode = 'LOITER'
        if mode not in master.mode_mapping():
            print('Unknown mode : {}'.format(mode))
            print('Try:', list(master.mode_mapping().keys()))
            sys.exit(1)
        mode_id = master.mode_mapping()[mode]
        master.set_mode(mode_id)
        while True:
            ack_msg = master.recv_match(type='COMMAND_ACK', blocking=True)
            ack_msg = ack_msg.to_dict()
            if ack_msg['command'] != mavutil.mavlink.MAV_CMD_DO_SET_MODE:
                # send acknowledgement back to console
                continue
            print(mavutil.mavlink.enums['MAV_RESULT'][ack_msg['result']].description)
            break

    # arming the drone
    if int(json_data['arm']) == 1:
        master.mav.command_long_send(master.target_system, master.target_component,
                                     mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
                                     0, 1, 0, 0, 0, 0, 0, 0)
        while True:
            msg = master.recv_msg()
            if msg and msg.get_type() == 'COMMAND_ACK' and msg.command == mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM:
                if msg.result == mavutil.mavlink.MAV_RESULT_ACCEPTED:
                    # send acknowledgement back to console
                    break
                else:
                    # send acknowledgement back to console
                    # exit()
                    break

    # disarming the drone
    if int(json_data['arm']) == 0:
        master.mav.command_long_send(
            master.target_system, master.target_component, mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
            0, 0,
            21196, 0,
            0, 0, 0, 0
        )

        while True:
            msg = master.recv_msg()
            if msg and msg.get_type() == 'COMMAND_ACK' and msg.command == mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM:
                if msg.result == mavutil.mavlink.MAV_RESULT_ACCEPTED:
                    # send acknowledgement back to console
                    break
                else:
                    # send acknowledgement back to console
                    break

    # Send control commands
    master.mav.manual_control_send(
        master.target_system,
        roll, pitch, -throttle, yaw,
        0)  # set to 0 to indicate that buttons are not being used

    time.sleep(1)  # sleep for a short time to avoid overloading the serial port


# when an error occurs with the WebSocket connection
def on_error(ws, error):
    print(error)


# when the WebSocket connection is closed
def on_close(ws, close_status_code, close_msg):
    print("Connection closed")


# when the WebSocket connection is opened
def on_open(ws):
    print("Connection opened")


if __name__ == "__main__":
    websocket.enableTrace(True)
    ws = websocket.WebSocketApp("wss://example.com/ws",
                                on_open=on_open,
                                on_message=on_message,
                                on_error=on_error,
                                on_close=on_close)
    ws.on_open = on_open
    ws.run_forever(dispatcher=rel,
                   reconnect=5)  # Set dispatcher to automatic reconnection, 5 second reconnect delay if connection closed unexpectedly
    rel.signal(2, rel.abort)  # Keyboard Interrupt
    rel.dispatch()
