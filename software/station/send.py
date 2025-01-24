import socket
import json
import time

TCP_IP = '192.168.4.1'
TCP_PORT = 3333

with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as soc:
    soc.connect((TCP_IP, TCP_PORT))

    while True:
        data = {
            'command': 'manual',
            'ref_cfg': [
                1500,
                1500,
                0,
                1500,
                1500,
                0,
            ],
        }
        soc.sendall(bytes(json.dumps(data), encoding='utf-8'))

        time.sleep(0.05)
