import time
import requests

IP = '192.168.4.1'

begin = time.time()

while True:

    response = requests.get(f'http://{IP}/get')
    print(response.text)

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
    requests.post(f'http://{IP}/post', json=data)

    time.sleep(0.05)
