import zmq
import time
import numpy as np
import sympy as sp
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

t, a, f = sp.symbols('t a f')
w = 2*np.pi*f
x = a*sp.cos(w*t)*sp.sin(w*t)/(1 + sp.sin(w*t)**2)
y = a*sp.cos(w*t)/(1 + sp.sin(w*t)**2)
theta = sp.atan2(y.diff(t), x.diff(t)) - sp.pi/4
trajectory = sp.Matrix([
    x, y, theta,
    x.diff(t), y.diff(t), theta.diff(t),
    x.diff(t, 2), y.diff(t, 2), theta.diff(t, 2),
])
trajectory = trajectory.subs([
    (a, 2),
    (f, 0.1),
])
trajectory = sp.lambdify(t, trajectory)

fig = plt.figure()
grid = gridspec.GridSpec(17, 3)
ax = plt.subplot(grid[0:10, :])

time_0 = time.time()

def periodic():
    data = {
        'trajectory_continue': None
    }
    publisher.send_json(data)

timer = fig.canvas.new_timer(interval=20)
timer.add_callback(periodic)

def reset_ekf(event):
    data = {'reset': None}
    publisher.send_json(data)

period = 10
total = 1000
traj_t = np.linspace(0, period, total)

def write_trajectory(event):
    for i in range(len(traj_t)):
        traj = np.array(trajectory(traj_t[i])).flatten()
        data = {
            'trajectory_write': {
                'total': int(total),
                'dt': period/total,
                'start': i,
                'nodes': [
                    [
                        traj[0],
                        traj[1],
                        traj[2],
                        -300,
                        +300,
                        traj[3],
                        traj[4],
                        traj[5],
                        0,
                        0,
                        traj[6],
                        traj[7],
                        traj[8],
                        0,
                        0,
                    ],
                ],
            },
        }
        publisher.send_json(data)
        print(f'transmit {i} nodes')
        time.sleep(0.01)

def read_trajectory(event):
    data = {
        'trajectory_read': None,
    }
    publisher.send_json(data)
    readed_x.clear()
    readed_y.clear()
    readed.set_xdata(readed_x)
    readed.set_ydata(readed_y)

def start_tracking(event):
    global time_0
    time_0 = time.time()
    timer.start()

def stop_tracking(event):
    timer.stop()

button_reset_ax = plt.subplot(grid[12, 1])
button_reset = Button(button_reset_ax, 'reset')
button_reset.on_clicked(reset_ekf)

button_write_ax = plt.subplot(grid[13, 1])
button_write = Button(button_write_ax, 'write trajectory')
button_write.on_clicked(write_trajectory)

button_read_ax = plt.subplot(grid[14, 1])
button_read = Button(button_read_ax, 'read trajectory')
button_read.on_clicked(read_trajectory)

button_start_ax = plt.subplot(grid[15, 1])
button_start = Button(button_start_ax, 'start')
button_start.on_clicked(start_tracking)

button_stop_ax = plt.subplot(grid[16, 1])
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

        x = telemetry['controller']['trajectory']['pos'][0]
        y = telemetry['controller']['trajectory']['pos'][1]
        theta = telemetry['controller']['trajectory']['pos'][2]

        if x and y and theta:
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
