import socket
import json

IP = '192.168.4.1'
PORT = 3333

with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as sock:
    data = {
        'config': {
            'accelerometer': [1, 2, 3, 4, 5, 6, 7, 810, 11],
            'gyroscope': [50],
        }
    }
    sock.sendto(bytes(json.dumps(data), encoding='utf-8'), (IP, PORT))
