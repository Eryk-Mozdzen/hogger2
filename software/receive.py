import zmq

context = zmq.Context()

socket = context.socket(zmq.SUB)

socket.connect("tcp://localhost:6000")

socket.setsockopt(zmq.SUBSCRIBE, b'')

try:
    while True:
        message = socket.recv()
        print(message.decode())

except KeyboardInterrupt:
    print("\nInterrupt received, stopping...")
finally:
    socket.close()
    context.term()
