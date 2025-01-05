import time
import requests

IP = '192.168.4.1'

begin = time.time()

while True:

    response = requests.get(f'http://{IP}/get')
    print(response.text)

    data = {
        'number': float(time.time() - begin),
        'boolean': True,
        'string': 'witajcie w mojej kuchni',
        'zero': None,
    }
    requests.post(f'http://{IP}/post', json=data)

    time.sleep(0.01)
