import asyncio
import json
import websockets
import evdev


async def websocket_client():
    # Specify the joystick device path
    joystick_path = '/dev/input/event1'

    while True:
        try:
            # Connect to the WebSocket server
            async with websockets.connect('ws://172.17.0.5:8765', ping_interval=None) as websocket:
                try:
                    # Open the joystick device
                    joystick = evdev.InputDevice(joystick_path)
                except FileNotFoundError:
                    print(f"Joystick device '{joystick_path}' not found")
                    return

                async for event in joystick.async_read_loop():
                    if event.type in (evdev.ecodes.EV_KEY, evdev.ecodes.EV_ABS):
                        # Create a JSON payload
                        payload = {
                            "arm": bool(event.code == evdev.ecodes.BTN_A and event.value == 1),
                            "disarm": bool(event.code == evdev.ecodes.BTN_B and event.value == 1),
                            "mode": bool(event.code == evdev.ecodes.BTN_X and event.value == 1),
                            "yaw": 0.0,
                            "throttle": 0.0,
                            "roll": 0.0,
                            "pitch": 0.0
                        }

                        if event.type == evdev.ecodes.EV_ABS:
                            if event.code == evdev.ecodes.ABS_X:
                                payload["yaw"] = event.value / 32767.0
                            elif event.code == evdev.ecodes.ABS_Y:
                                payload["throttle"] = event.value / 32767.0
                            elif event.code == evdev.ecodes.ABS_RX:
                                payload["roll"] = event.value / 32767.0
                            elif event.code == evdev.ecodes.ABS_RY:
                                payload["pitch"] = event.value / 32767.0

                        payload_json = json.dumps(payload)

                        # Send the JSON payload to the server
                        await websocket.send(payload_json)
                        print("Sent JSON data:", payload)

                        # Receive and process the response from the server
                        response_json = await websocket.recv()
                        response = json.loads(response_json)
                        print("Received response:", response)

        except websockets.ConnectionClosedError:
            print("Connection closed. Reconnecting...")
        except websockets.WebSocketException:
            print("WebSocket error occurred. Reconnecting...")
        except Exception as e:
            print(f"An error occurred: {e}. Reconnecting...")

        # Sleep for a short time before attempting to reconnect
        await asyncio.sleep(1)


if __name__ == '__main__':
    asyncio.run(websocket_client())
