import socket

IP = '0.0.0.0'
PORT = 4444

sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock.bind((IP, PORT))

while True:
    data, _ = sock.recvfrom(1024)

    print(data.decode())
