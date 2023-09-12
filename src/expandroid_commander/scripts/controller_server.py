import socket


SERVER_PORT = 13579

sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
# broadcast
sock.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
sock.bind(("", SERVER_PORT))

while True:
    data, address = sock.recvfrom(4096)
    print(data.decode("utf-8"))
