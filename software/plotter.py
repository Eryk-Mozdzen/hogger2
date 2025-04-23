import sys
import json
import matplotlib.pyplot as plt
import numpy as np
from collections import defaultdict

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

data_raw = make_recursive_list_dict()

with open(sys.argv[1], 'r') as file:
    for line in file:
        json_obj = json.loads(line)
        telemetry = json_obj.get('telemetry', {})
        if telemetry.get('controller', {}).get('started', False):
            collect_values(data_raw, telemetry)

data = convert_to_normal_dict(data_raw)

data['timestamp'] = [(t - data['timestamp'][0])*1e-6 for t in data['timestamp']]

plt.figure()
plt.subplot(2, 1, 1)
plt.plot(data['timestamp'], data['trajectory']['href'][0], label='$x$ reference', linestyle='dashed', color='red')
plt.plot(data['timestamp'], data['controller']['h'][0], label='$x$', color='red')
plt.plot(data['timestamp'], data['trajectory']['href'][1], label='$y$ reference', linestyle='dashed', color='green')
plt.plot(data['timestamp'], data['controller']['h'][1], label='$y$', color='green')
plt.title('reference tracking')
plt.xlabel('t [s]')
plt.ylabel('[m]')
plt.grid()
plt.legend()
plt.subplot(2, 1, 2)
plt.plot(data['timestamp'], data['trajectory']['href'][2], label='$\\theta$ reference', linestyle='dashed', color='blue')
plt.plot(data['timestamp'], data['controller']['h'][2], label='$\\theta$', color='blue')
plt.xlabel('t [s]')
plt.ylabel('[rad]')
plt.grid()
plt.legend()

plt.figure()
plt.subplot(2, 1, 1)
plt.plot(data['timestamp'], np.array(data['controller']['h'][0]) - np.array(data['trajectory']['href'][0]), label='$e_x$', color='red')
plt.plot(data['timestamp'], np.array(data['controller']['h'][1]) - np.array(data['trajectory']['href'][1]), label='$e_y$', color='green')
plt.title('reference tracking error')
plt.xlabel('t [s]')
plt.ylabel('error [m]')
plt.grid()
plt.legend()
plt.subplot(2, 1, 2)
plt.plot(data['timestamp'], np.array(data['controller']['h'][2]) - np.array(data['trajectory']['href'][2]), label='$e_\\theta$', color='blue')
plt.xlabel('t [s]')
plt.ylabel('error [rad]')
plt.grid()
plt.legend()

plt.figure()
plt.subplot(2, 1, 1)
plt.plot(data['timestamp'], data['motor_1']['vel_ref'], label='$\\dot{\\psi}_1$ reference', linestyle='dashed', color='cyan')
plt.plot(data['timestamp'], data['motor_1']['vel'], label='$\\dot{\\psi}_1$', color='cyan')
plt.plot(data['timestamp'], data['motor_2']['vel_ref'], label='$\\dot{\\psi}_2$ reference', linestyle='dashed', color='magenta')
plt.plot(data['timestamp'], data['motor_2']['vel'], label='$\\dot{\\psi}_2$', color='magenta')
plt.title('control signals')
plt.xlabel('t [s]')
plt.ylabel('HOG velocity [rad/s]')
plt.legend()
plt.grid()
plt.subplot(2, 1, 2)
plt.plot(data['timestamp'], np.degrees(data['servo_1_x']['pos']), label='$\\phi_1$')
plt.plot(data['timestamp'], np.degrees(data['servo_1_y']['pos']), label='$\\theta_1$')
plt.plot(data['timestamp'], np.degrees(data['servo_2_x']['pos']), label='$\\phi_2$')
plt.plot(data['timestamp'], np.degrees(data['servo_2_y']['pos']), label='$\\theta_2$')
plt.xlabel('t [s]')
plt.ylabel('HOG attitude [deg]')
plt.legend()
plt.grid()

plt.show()
