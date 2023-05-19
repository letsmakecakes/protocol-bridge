import asyncio
import json
import websockets
import pygame


async def websocket_client():
    # Connect to the WebSocket server
    async with websockets.connect('ws://localhost:8765') as websocket:
        pygame.init()
        pygame.joystick.init()

        if pygame.joystick.get_count() == 0:
            print("No gamepad detected")
            return

        joystick = pygame.joystick.Joystick(0)
        joystick.init()

        while True:
            for _ in pygame.event.get():
                # Create a JSON payload
                payload = {"arm": joystick.get_button(0),
                           "disarm": joystick.get_button(2),
                           "mode": joystick.get_button(1),
                           "yaw": joystick.get_axis(0),
                           "throttle": joystick.get_axis(1),
                           "roll": joystick.get_axis(2),
                           "pitch": joystick.get_axis(3)}
                payload_json = json.dumps(payload)

                # Send the JSON payload to the server
                await websocket.send(payload_json)
                print("Sent JSON data:", payload)

                # Receive and process the response from the server
                response_json = await websocket.recv()
                response = json.loads(response_json)
                print("Received response:", response)


asyncio.get_event_loop().run_until_complete(websocket_client())