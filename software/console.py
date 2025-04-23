import zmq
import sys
import json

context = zmq.Context()
subscriber = context.socket(zmq.SUB)
subscriber.connect("tcp://localhost:6000")
subscriber.setsockopt(zmq.SUBSCRIBE, b'')

keys = []
if len(sys.argv)>1:
    keys = sys.argv[1:]

try:
    while True:
        message = subscriber.recv_json()
        if "telemetry" in message:
            if len(keys)>0:
                for key in keys:
                    if key in message["telemetry"]:
                        print(key + ': ' + json.dumps(message["telemetry"][key], indent=4))
            else:
                print(json.dumps(message, indent=4))

except KeyboardInterrupt:
    print("\nInterrupt received, stopping...")
finally:
    subscriber.close()
    context.term()
