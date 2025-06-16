import sys
import json
import os
import glob
import matplotlib.pyplot as plt
import numpy as np
from collections import defaultdict

plt.rcParams.update({
    "text.usetex": True,
    "font.family": "serif",
    "font.size": 12,
    "svg.fonttype": "none",
    "text.latex.preamble": r"\usepackage{newpxtext}\usepackage{eulervm}",
})

def make_recursive_list_dict():
    return defaultdict(make_recursive_list_dict)

def collect_values(store, obj):
    if isinstance(obj, dict):
        for k, v in obj.items():
            collect_values(store[k], v)
    elif isinstance(obj, list):
        for i, item in enumerate(obj):
            collect_values(store[i], item)
    else:
        if not isinstance(store, list):
            store = store.setdefault('data', [])
        store.append(obj)

def convert_to_normal_dict(d):
    if isinstance(d, defaultdict):
        d = {k: convert_to_normal_dict(v) for k, v in d.items()}
        if list(d.keys()) == ['data']:
            return d['data']
        return d
    return d

def recursive_filter(data_node, valid_indices, ref_len):
    if isinstance(data_node, dict):
        return {k: recursive_filter(v, valid_indices, ref_len) for k, v in data_node.items()}
    elif isinstance(data_node, list):
        if len(data_node) == ref_len:
            return [data_node[i] for i in valid_indices]
        else:
            return [recursive_filter(item, valid_indices, ref_len) for item in data_node]
    else:
        return data_node

data_raw = make_recursive_list_dict()

path = sys.argv[1]
file = os.path.basename(path)
filename, extension = os.path.splitext(file)

with open(path, 'r') as file:
    for line in file:
        json_obj = json.loads(line)
        telemetry = json_obj.get('telemetry', {})
        if telemetry.get('controller', {}).get('started', False):
            collect_values(data_raw, telemetry)

data = convert_to_normal_dict(data_raw)

data['timestamp'] = [(t - data['timestamp'][0])*1e-6 for t in data['timestamp']]

duration = 30

valid_indices = [i for i, t in enumerate(data['timestamp']) if t <= duration]
ref_len = len(data['timestamp'])
data = recursive_filter(data, valid_indices, ref_len)

path = 'plotter_output/' + filename
files = glob.glob(f'{path}/*')
for f in files:
    os.remove(f)
os.makedirs(path, exist_ok=True)

plt.figure()
plt.plot(data['trajectory'][0], data['trajectory'][1], linestyle='dashed', color='black')
plt.plot(data['estimate']['pos'][0], data['estimate']['pos'][1], color='red')
plt.xlabel('x [m]')
plt.ylabel('y [m]')
plt.grid()
plt.gca().set_aspect('equal')

xlim = plt.xlim()
ylim = plt.ylim()

x_range = xlim[1] - xlim[0]
y_range = ylim[1] - ylim[0]
side = max(x_range, y_range)

x_center = 0.5 * (xlim[0] + xlim[1])
y_center = 0.5 * (ylim[0] + ylim[1])

plt.xlim(x_center - side / 2, x_center + side / 2)
plt.ylim(y_center - side / 2, y_center + side / 2)

plt.savefig(f'{path}/tracking_xy_planar.svg', bbox_inches='tight', pad_inches=0)

plt.figure()
plt.plot(data['timestamp'], np.array(data['trajectory'][2]) + np.pi/4, linestyle='dashed', color='black')
plt.plot(data['timestamp'], np.array(data['estimate']['pos'][2]), color='blue')
plt.xlabel('time [s]')
plt.ylabel('$\\theta$ [rad]')
plt.grid()
plt.xlim(0, duration)
plt.savefig(f'{path}/tracking_theta.svg', bbox_inches='tight', pad_inches=0)

plt.figure()
plt.plot(data['timestamp'], np.array(data['estimate']['pos'][0]) - np.array(data['trajectory'][0]), label='$e_x$', color='red')
plt.plot(data['timestamp'], np.array(data['estimate']['pos'][1]) - np.array(data['trajectory'][1]), label='$e_y$', color='green')
plt.xlabel('time [s]')
plt.ylabel('error [m]')
plt.grid()
plt.legend()
plt.xlim(0, duration)
plt.savefig(f'{path}/error_xy.svg', bbox_inches='tight', pad_inches=0)

plt.figure()
plt.plot(data['timestamp'], np.array(data['estimate']['pos'][2]) - np.array(data['trajectory'][2]) - np.pi/4, label='$e_\\theta$', color='blue')
plt.xlabel('time [s]')
plt.ylabel('error [rad]')
plt.grid()
plt.xlim(0, duration)
plt.savefig(f'{path}/error_theta.svg', bbox_inches='tight', pad_inches=0)

plt.figure()
plt.plot(data['timestamp'], np.degrees(data['servos']['phi_1']['pos_ref']), linestyle='dashed', color='black')
plt.plot(data['timestamp'], np.degrees(data['servos']['phi_1']['pos']), color='red')
plt.xlabel('time [s]')
plt.ylabel('$\\phi_1$ [deg]')
plt.grid()
plt.xlim(0, duration)
plt.ylim(-5, 5)
plt.savefig(f'{path}/control_phi_1.svg', bbox_inches='tight', pad_inches=0)

plt.figure()
plt.plot(data['timestamp'], np.degrees(data['servos']['phi_2']['pos_ref']), linestyle='dashed', color='black')
plt.plot(data['timestamp'], np.degrees(data['servos']['phi_2']['pos']), color='blue')
plt.xlabel('time [s]')
plt.ylabel('$\\phi_2$ [deg]')
plt.grid()
plt.xlim(0, duration)
plt.ylim(-5, 5)
plt.savefig(f'{path}/control_phi_2.svg', bbox_inches='tight', pad_inches=0)

plt.figure()
plt.plot(data['timestamp'], np.degrees(data['servos']['theta_1']['pos_ref']), linestyle='dashed', color='black')
plt.plot(data['timestamp'], np.degrees(data['servos']['theta_1']['pos']), color='green')
plt.xlabel('time [s]')
plt.ylabel('$\\theta_1$ [deg]')
plt.grid()
plt.xlim(0, duration)
plt.ylim(-5, 5)
plt.savefig(f'{path}/control_theta_1.svg', bbox_inches='tight', pad_inches=0)

plt.figure()
plt.plot(data['timestamp'], np.degrees(data['servos']['theta_2']['pos_ref']), linestyle='dashed', color='black')
plt.plot(data['timestamp'], np.degrees(data['servos']['theta_2']['pos']), color='orange')
plt.xlabel('time [s]')
plt.ylabel('$\\theta_2$ [deg]')
plt.grid()
plt.xlim(0, duration)
plt.ylim(-5, 5)
plt.savefig(f'{path}/control_theta_2.svg', bbox_inches='tight', pad_inches=0)

plt.figure()
plt.plot(data['timestamp'], data['motor_1']['vel_ref'], linestyle='dashed', color='red')
plt.plot(data['timestamp'], data['motor_1']['vel'], label='$\\dot{\\psi}_1$', color='red')
plt.plot(data['timestamp'], data['motor_2']['vel_ref'], linestyle='dashed', color='green')
plt.plot(data['timestamp'], data['motor_2']['vel'], label='$\\dot{\\psi}_2$', color='green')
plt.xlabel('time [s]')
plt.ylabel('$\\dot{\\psi}$ [rad/s]')
plt.grid()
plt.xlim(0, duration)
plt.legend()
plt.savefig(f'{path}/control_dot_psi_12.svg', bbox_inches='tight', pad_inches=0)
