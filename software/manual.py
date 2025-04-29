import pygame
import zmq
import time
import datetime
import numpy

context = zmq.Context()
subscriber = context.socket(zmq.SUB)
subscriber.connect("tcp://localhost:6000")
subscriber.setsockopt(zmq.SUBSCRIBE, b'')
publisher = context.socket(zmq.PUB)
publisher.connect("tcp://localhost:7000")

pygame.init()
joystick = pygame.joystick.Joystick(0)
joystick.init()

next_config = datetime.datetime.now()

try:
    while True:
        pygame.event.pump()

        L = 0.13
        R = 0.05
        W = 300

        dx_ref = -1*joystick.get_axis(1)
        dtheta_ref = 2*joystick.get_axis(0)

        v1 = dx_ref + L*dtheta_ref
        v2 = dx_ref - L*dtheta_ref

        a1 = numpy.arcsin(numpy.clip(v1/(+W*R), -1., 1.))
        a2 = numpy.arcsin(numpy.clip(v2/(-W*R), -1., 1.))

        data = {
            'manual_servo': [
                a1,
                0,
                a2,
                0,
            ],
        }
        #print(data)
        publisher.send_json(data)

        data = {
            'manual_motor': [
                -W if joystick.get_axis(2)>0 else 0,
                +W if joystick.get_axis(2)>0 else 0,
            ],
        }
        #print(data)
        publisher.send_json(data)

        #print([round(joystick.get_axis(i), 2) for i in range(joystick.get_numaxes())])
        #print([joystick.get_button(i) for i in range(joystick.get_numbuttons())])
        #print([joystick.get_hat(i) for i in range(joystick.get_numhats())])

        if datetime.datetime.now() > next_config:
            next_config = datetime.datetime.now() + datetime.timedelta(milliseconds=500)
            data = {
                'config_req': None,
            }
            #print(data)
            publisher.send_json(data)

        config = None

        while True:
            try:
                message = subscriber.recv_json(flags=zmq.NOBLOCK)
                if 'config' in message:
                    config = message
            except zmq.Again:
                break

        if config:
            if isinstance(config.get('config'), dict):
                print(config['config']['servo_offset'])

                if not 'servo_offset' in config['config']:
                    config['config']['servo_offset'] = [0, 0, 0, 0]

                if joystick.get_hat(0)[1]==1:
                    config['config']['servo_offset'][1] +=numpy.deg2rad(0.1)
                    config['config']['servo_offset'][3] +=numpy.deg2rad(0.1)
                    publisher.send_json(config)

                if joystick.get_hat(0)[1]==-1:
                    config['config']['servo_offset'][1] -=numpy.deg2rad(0.1)
                    config['config']['servo_offset'][3] -=numpy.deg2rad(0.1)
                    publisher.send_json(config)

                if joystick.get_button(1):
                    config['config']['servo_offset'][0] +=numpy.deg2rad(0.1)
                    config['config']['servo_offset'][2] +=numpy.deg2rad(0.1)
                    publisher.send_json(config)

                if joystick.get_button(2):
                    config['config']['servo_offset'][0] -=numpy.deg2rad(0.1)
                    config['config']['servo_offset'][2] -=numpy.deg2rad(0.1)
                    publisher.send_json(config)

                if joystick.get_button(0):
                    config['config']['servo_offset'][0] -=numpy.deg2rad(0.1)
                    config['config']['servo_offset'][2] +=numpy.deg2rad(0.1)
                    publisher.send_json(config)

                if joystick.get_button(3):
                    config['config']['servo_offset'][0] +=numpy.deg2rad(0.1)
                    config['config']['servo_offset'][2] -=numpy.deg2rad(0.1)
                    publisher.send_json(config)

        time.sleep(0.05)

except KeyboardInterrupt:
    pygame.quit()
    context.destroy()
