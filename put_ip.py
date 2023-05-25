import socket

# Get the system's IP address
ip_address = socket.gethostbyname(socket.gethostname())

# Store the IP address in a text file
with open('ip_address.txt', 'w') as file:
    file.write(ip_address)