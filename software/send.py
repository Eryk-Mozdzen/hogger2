import socket
import json
import time

IP = '192.168.4.1'
PORT = 3333

with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as sock:
    while True:
        data = {
            'reference': [
                0,
                0,
                500,
                0,
                0,
                0,
            ],
        }
        sock.sendto(bytes(json.dumps(data), encoding='utf-8'), (IP, PORT))

        time.sleep(0.05)
