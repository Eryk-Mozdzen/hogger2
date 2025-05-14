import zmq
import json
import datetime
import sys

context = zmq.Context()
subscriber = context.socket(zmq.SUB)
subscriber.connect("tcp://localhost:6000")
subscriber.setsockopt(zmq.SUBSCRIBE, b'')

filename = datetime.datetime.now().strftime('%Y_%m_%d_%H_%M_%S.ndjson')

if len(sys.argv)>1:
    filename = sys.argv[1] + '.ndjson'

try:
    with open(filename, 'w') as file:
        while True:
            message = subscriber.recv_json()
            if "telemetry" in message:
                file.write(json.dumps(message) + '\n')
                file.flush()

except KeyboardInterrupt:
    print("\nInterrupt received, stopping...")
finally:
    subscriber.close()
    context.term()
