import asyncio
import json
import websockets


async def websocket_client():
    while True:
        try:
            # Connect to the WebSocket server
            async with websockets.connect('ws://172.17.0.4:8765', ping_interval=None) as websocket:
                while True:
                    # Get input from alternative sources (e.g., keyboard or mouse)
                    arm = input("Arm button (0 or 1): ")
                    disarm = input("Disarm button (0 or 1): ")
                    mode = input("Mode button (0 or 1): ")
                    yaw = input("Yaw axis (-1.0 to 1.0): ")
                    throttle = input("Throttle axis (-1.0 to 1.0): ")
                    roll = input("Roll axis (-1.0 to 1.0): ")
                    pitch = input("Pitch axis (-1.0 to 1.0): ")

                    # Create a JSON payload
                    payload = {
                        "arm": int(arm),
                        "disarm": int(disarm),
                        "mode": int(mode),
                        "yaw": float(yaw),
                        "throttle": float(throttle),
                        "roll": float(roll),
                        "pitch": float(pitch)
                    }
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
