import socket
import subprocess

# Get system's IP address
system_ip = socket.gethostbyname(socket.gethostname())

# Store the IP address in a text file
with open('ip_address.txt', 'w') as file:
    file.write(system_ip)

# Replace <system-ipaddress> in the command
command = [
    'mavproxy.py',
    '--master=/dev/ttyACM0',
    '--baudrate',
    '921600',
    '--out',
    '127.0.0.1:14550',
    '--out',
    f'udp:{system_ip}:14550'
]

subprocess.run(command)
