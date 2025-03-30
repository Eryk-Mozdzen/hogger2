import pygame
import zmq
import time
import numpy
import shelve

context = zmq.Context()
publisher = context.socket(zmq.PUB)
publisher.connect("tcp://localhost:7000")

pygame.init()
joystick = pygame.joystick.Joystick(0)
joystick.init()

with shelve.open('prefs') as db:
    offset = db.get('offset', [0, 0])

try:
    while True:
        pygame.event.pump()

        L = 0.13
        R = 0.05
        W = 300

        dx_ref = -1*(joystick.get_axis(1) + offset[0])
        dtheta_ref = 2*(joystick.get_axis(0) + offset[1])

        v1 = dx_ref + L*dtheta_ref
        v2 = dx_ref - L*dtheta_ref

        a1 = numpy.arcsin(numpy.clip(v1/(+W*R), -1., 1.))
        a2 = numpy.arcsin(numpy.clip(v2/(-W*R), -1., 1.))

        data = {
            'manual': [
                a1,
                0,
                -W if joystick.get_axis(2)>0 else 0,
                a2,
                0,
                +W if joystick.get_axis(2)>0 else 0,
            ],
        }

        print(data)
        print([round(joystick.get_axis(i), 2) for i in range(joystick.get_numaxes())])
        print([joystick.get_button(i) for i in range(joystick.get_numbuttons())])

        publisher.send_json(data)

        offset[0] +=(0.05 if joystick.get_button(0) else 0)
        offset[0] -=(0.05 if joystick.get_button(3) else 0)
        offset[1] +=(0.05 if joystick.get_button(1) else 0)
        offset[1] -=(0.05 if joystick.get_button(2) else 0)

        with shelve.open('prefs') as db:
            db['offset'] = offset

        time.sleep(0.05)

except KeyboardInterrupt:
    pygame.quit()
    context.destroy()
