import socket

UDP_IP = "0.0.0.0"
UDP_PORT = 4444

sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock.bind((UDP_IP, UDP_PORT))

while True:
    data, _ = sock.recvfrom(1024)

    print(data.decode())
