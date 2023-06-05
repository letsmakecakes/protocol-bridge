# Protocol Bridge for Joystick Drone Control

This repository contains a protocol bridge for controlling a drone using a joystick. The bridge allows communication between a client, which reads input from a joystick and sends control commands to the drone, and a server, which receives the commands and sends them to the flight controller.

## Requirements

To use this protocol bridge, you need the following:

- Python 3.x
- [Pygame](https://www.pygame.org) library
- [websockets](https://pypi.org/project/websockets/) library
- [pymavlink](https://pypi.org/project/pymavlink/) library

## Installation

1. Clone this repository:

   ```bash
   git clone https://github.com/letsmakecakes/protocol-bridge.git
   ```

2. Install the required libraries. Run the following command:

   ```bash
   pip install pygame websockets pymavlink
   ```

## Usage

### Client

1. Open the `client.py` file in a text editor.

2. Modify the following line to match the address and port of the server:

   ```python
   async with websockets.connect('ws://localhost:8765', ping_interval=None) as websocket:
   ```

3. Connect your joystick to the computer.

4. Run the client script using the following command:

   ```bash
   python client.py
   ```

### Server

1. Open the `server.py` file in a text editor.

2. Modify the following line to match the connection string of your flight controller:

   ```python
   master = mavutil.mavlink_connection('<your-system-ip-address>:14550')
   ```

3. Run the server script using the following command:

   ```bash
   python server.py
   ```

## Communication Protocol

The client reads input from the joystick and sends control commands to the server using a WebSocket connection. The server receives the commands and sends them to the flight controller using the MAVLink protocol.

The control commands include the following parameters:

- `arm`: Arm the flight controller (value: `1` to arm, `0` to disarm).
- `disarm`: Disarm the flight controller (value: `1` to disarm, `0` to keep armed).
- `mode`: Set the flight mode of the drone (value: `1` to set mode, `0` to keep the current mode).
- `yaw`: Yaw control value (-1.0 to 1.0).
- `throttle`: Throttle control value (-1.0 to 1.0).
- `roll`: Roll control value (-1.0 to 1.0).
- `pitch`: Pitch control value (-1.0 to 1.0).

The server receives the control commands and processes them accordingly:

- If `arm` is set to `1`, the server arms the flight controller.
- If `disarm` is set to `1`, the server disarms the flight controller.
- If `mode` is set to `1`, the server sets the flight mode of the drone to the specified mode.

The server then sends the control commands to the flight controller using the MAVLink protocol. The control values for yaw, throttle, roll, and pitch are scaled and sent as manual control commands.

## License

This project is licensed under the [MIT License](LICENSE).

Feel free to contribute to this project by submitting issues or pull requests.
