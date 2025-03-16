import zmq
import json
import socket
import numpy

context = zmq.Context()
subscriber = context.socket(zmq.SUB)
subscriber.connect("tcp://localhost:6000")
subscriber.setsockopt(zmq.SUBSCRIBE, b'')

soc = socket.socket()
soc.connect(('localhost', 8080))

def write(msg):
    soc.send(json.dumps(msg).encode())

write({
    'command': 'clear',
})
write({
    'command': 'config',
    'theme': 'light',
    'camera': 'orbit',
})
write({
    'command': 'create',
    'path': 'obj1',
    #'geometry': {
    #    'shape': 'model',
    #    'file': '/home/emozdzen/repos/hogger2/software/hogger2.stl',
    #},
    'geometry': {
        'shape': 'cuboid',
        'size': [0.15, 0.4, 0.05]
    },
    'material': {
        'color': [128, 128, 128]
    },
})

try:
    while True:
        message = json.loads(subscriber.recv().decode())

        if 'telemetry' in message:
            write({
                'command': 'update',
                'path': 'obj1',
                'transform': {
                    'translation': [
                        message['telemetry']['estimate']['pos'][0],
                        message['telemetry']['estimate']['pos'][1],
                        0,
                    ],
                    'rpy': [
                        0,
                        0,
                        numpy.rad2deg(message['telemetry']['estimate']['theta']),
                    ],
                },
            })

except KeyboardInterrupt:
    print("\nInterrupt received, stopping...")
finally:
    subscriber.close()
    context.term()
