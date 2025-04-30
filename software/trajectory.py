import zmq
import time
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.animation as animation
import matplotlib.gridspec as gridspec
from matplotlib.patches import Polygon
from matplotlib.widgets import Button

context = zmq.Context()
subscriber = context.socket(zmq.SUB)
subscriber.connect("tcp://localhost:6000")
subscriber.setsockopt(zmq.SUBSCRIBE, b'')
publisher = context.socket(zmq.PUB)
publisher.connect("tcp://localhost:7000")

fig = plt.figure()
grid = gridspec.GridSpec(18, 3)
ax = plt.subplot(grid[0:10, :])

time_0 = time.time()

motors_active = False
controller_active = False

def periodic():
    if motors_active:
        data = {
            'manual_motor': [
                -200,
                +200,
            ],
        }
        publisher.send_json(data)
    if motors_active and not controller_active:
        data = {
            'manual_servo': [
                0,
                0,
                0,
                0,
            ],
        }
        publisher.send_json(data)
    if controller_active:
        data = {
            'controller_continue': None
        }
        publisher.send_json(data)

timer = fig.canvas.new_timer(interval=100)
timer.add_callback(periodic)
timer.start()

def reset_ekf(event):
    data = {'reset': None}
    publisher.send_json(data)

def write_trajectory(event):
    data = {
        'trajectory_write': {
            'generator': 'circle',
            'params': [
                0,
                0,
                1,
                20,
            ],
        },
    }
    publisher.send_json(data)

def read_trajectory(event):
    data = {
        'trajectory_read': 20,
    }
    publisher.send_json(data)
    readed_x.clear()
    readed_y.clear()
    readed.set_xdata(readed_x)
    readed.set_ydata(readed_y)

def start_motors(event):
    global motors_active
    global time_0
    motors_active = True

def start_tracking(event):
    global controller_active
    global time_0
    time_0 = time.time()
    controller_active = True

def stop_tracking(event):
    global motors_active
    global controller_active
    motors_active = False
    controller_active = False
    for _ in range(10):
        data = {
            'stop': None,
        }
        publisher.send_json(data)

button_reset_ax = plt.subplot(grid[12, 1])
button_reset = Button(button_reset_ax, 'reset')
button_reset.on_clicked(reset_ekf)

button_write_ax = plt.subplot(grid[13, 1])
button_write = Button(button_write_ax, 'write trajectory')
button_write.on_clicked(write_trajectory)

button_read_ax = plt.subplot(grid[14, 1])
button_read = Button(button_read_ax, 'read trajectory')
button_read.on_clicked(read_trajectory)

button_start_motors_ax = plt.subplot(grid[15, 1])
button_start_motors = Button(button_start_motors_ax, 'start motors')
button_start_motors.on_clicked(start_motors)

button_start_tracking_ax = plt.subplot(grid[16, 1])
button_start_tracking = Button(button_start_tracking_ax, 'start tracking')
button_start_tracking.on_clicked(start_tracking)

button_stop_ax = plt.subplot(grid[17, 1])
button_stop = Button(button_stop_ax, 'stop')
button_stop.on_clicked(stop_tracking)

size = 0.5
base_triangle = np.array([
    [-0.25*size, -0.35*size],
    [-0.25*size, 0.35*size],
    [0.75*size, 0.],
])

readed_x = []
readed_y = []
readed, = ax.plot(readed_x, readed_y, 'k--', zorder=1)

reference = Polygon(base_triangle, closed=True, fc='red', ec='black', label='reference pose', zorder=2)
estimated = Polygon(base_triangle, closed=True, fc='cyan', ec='black', label='estimated pose', zorder=2)

ax.add_patch(reference)
ax.add_patch(estimated)

ax.set_xlim(-3, 3)
ax.set_ylim(-3, 3)
ax.set_aspect('equal')
ax.grid()
ax.legend()

def update(frame):
    telemetry = None
    trajectory = None

    while True:
        try:
            message = subscriber.recv_json(flags=zmq.NOBLOCK)
            if 'telemetry' in message:
                telemetry = message['telemetry']

            if 'trajectory' in message:
                trajectory = message['trajectory']
        except zmq.Again:
            break

    if telemetry:
        x = telemetry['estimate']['pos'][0]
        y = telemetry['estimate']['pos'][1]
        theta = telemetry['estimate']['pos'][2]

        if x and y and theta:
            c, s = np.cos(theta), np.sin(theta)
            rotation_matrix = np.array([[c, -s], [s, c]])
            rotated = np.dot(base_triangle, rotation_matrix.T)
            translated = rotated + np.array([x, y])
            estimated.set_xy(translated)

        x = telemetry['trajectory'][0]
        y = telemetry['trajectory'][1]
        theta = telemetry['trajectory'][2]

        c, s = np.cos(theta), np.sin(theta)
        rotation_matrix = np.array([[c, -s], [s, c]])
        rotated = np.dot(base_triangle, rotation_matrix.T)
        translated = rotated + np.array([x, y])
        reference.set_xy(translated)

    if trajectory:
        readed_x.append(trajectory['node'][0])
        readed_y.append(trajectory['node'][1])

        readed.set_xdata(readed_x)
        readed.set_ydata(readed_y)

    return estimated, reference, readed,

anim = animation.FuncAnimation(fig, update, interval=20, blit=True)

plt.show()

subscriber.close()
publisher.close()
context.term()
