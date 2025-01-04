import time
import requests

IP = '192.168.4.1'

while True:

    response = requests.get(f'http://{IP}/get')
    print(response.text)

    data = {
        'PC timestamp': int(time.time())
    }
    requests.post(f'http://{IP}/post', json=data)

    time.sleep(0.01)
