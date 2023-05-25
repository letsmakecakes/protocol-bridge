import time

import sensor as sensor
from pymavlink import mavutil


def wait_conn(device):
    """ Sends a ping to develop UDP communication and waits for a response. """
    msg = None
    global boot_time
    while not msg:
        # boot_time = time.time()
        device.mav.ping_send(
            int((time.time() - boot_time) * 1e6),  # Unix time since boot (microseconds)
            0,  # Ping number
            0,  # Request ping of all systems
            0  # Request ping of all components
        )
        msg = device.recv_match()
        time.sleep(0.5)


if __name__ == '__main__':
    boot_time = time.time()
    # establish connection to top-side computer
    direction = 'udpout'
    device_ip = '192.168.2.1'
    port = '14550'
    params = [direction, device_ip, port]
    print('connecting to {1} via {0} on port {2}'.format(*params))
    computer = mavutil.mavlink_connection(':'.join(params), source_system=1)
    print('waiting for confirmation...')
    wait_conn(computer)
    print('connection success!')

    # connect to sensor and set up output
    # TODO: connect to sensor
    sensor_name = 'MySensor'  # MUST be 9 characters or fewer

    while 'reading data':
        value = sensor.get_value()  # TODO: implement something like this
        computer.mav.named_value_float_send(
            int((time.time() - boot_time) * 1e3),  # Unix time since boot (milliseconds)
            sensor_name,
            value
        )