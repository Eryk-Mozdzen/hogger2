import zmq
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

fig = plt.figure()
grid = gridspec.GridSpec(16, 3)
ax = plt.subplot(grid[0:11, :])

def periodic():
    pass

timer = fig.canvas.new_timer(interval=20)
timer.add_callback(periodic)

def reset_ekf(event):
    pass

def start_tracking(event):
    timer.start()

def stop_tracking(event):
    timer.stop()

button_reset_ax = plt.subplot(grid[13, 1])
button_reset = Button(button_reset_ax, 'reset')
button_reset.on_clicked(reset_ekf)

button_start_ax = plt.subplot(grid[14, 1])
button_start = Button(button_start_ax, 'start')
button_start.on_clicked(start_tracking)

button_stop_ax = plt.subplot(grid[15, 1])
button_stop = Button(button_stop_ax, 'stop')
button_stop.on_clicked(start_tracking)

size = 0.5
base_triangle = np.array([
    [-0.25*size, -0.35*size],
    [-0.25*size, 0.35*size],
    [0.75*size, 0.],
])
triangle = Polygon(base_triangle, closed=True, fc='cyan', ec='black', label='estimated pose')
ax.add_patch(triangle)

ax.set_xlim(-3, 3)
ax.set_ylim(-3, 3)
ax.set_aspect('equal')
ax.grid()
ax.legend()

def update(frame):
    telemetry = None

    while True:
        try:
            message = subscriber.recv_json(flags=zmq.NOBLOCK)
            if 'telemetry' in message:
                telemetry = message['telemetry']
        except zmq.Again:
            break

    if telemetry:
        x = telemetry['estimate']['pos'][0]
        y = telemetry['estimate']['pos'][1]
        theta = telemetry['estimate']['theta']

        c, s = np.cos(theta), np.sin(theta)
        rotation_matrix = np.array([[c, -s], [s, c]])

        rotated = np.dot(base_triangle, rotation_matrix.T)
        translated = rotated + np.array([x, y])

        triangle.set_xy(translated)

    return triangle,

anim = animation.FuncAnimation(fig, update, interval=20, blit=True)

plt.show()

subscriber.close()
context.term()
