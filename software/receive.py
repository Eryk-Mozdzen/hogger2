import zmq

context = zmq.Context()
subscriber = context.socket(zmq.SUB)
subscriber.connect("tcp://localhost:6000")
subscriber.setsockopt(zmq.SUBSCRIBE, b'')

try:
    while True:
        message = subscriber.recv().decode()
        print(message)

except KeyboardInterrupt:
    print("\nInterrupt received, stopping...")
finally:
    subscriber.close()
    context.term()
