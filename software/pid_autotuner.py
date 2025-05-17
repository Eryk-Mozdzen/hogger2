import sys
import json
import matplotlib.pyplot as plt
import numpy as np
from collections import defaultdict
import scipy.optimize

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
        if sys.argv[2]!='controller' or telemetry.get('controller', {}).get('started', False):
            collect_values(data_raw, telemetry)

data = convert_to_normal_dict(data_raw)

data['timestamp'] = [(t - data['timestamp'][0])*1e-6 for t in data['timestamp']]

if sys.argv[2]=='controller':
    plt.figure()
    plt.plot(data['timestamp'], data['controller']['exp_step'], label='step')
    plt.plot(data['timestamp'], data['controller']['exp_response'], label='step response')
    plt.grid()
    plt.legend()

    step_start = np.nonzero(data['controller']['exp_step'])[0][0]
    step_value = max(data['controller']['exp_step'])

    duration = 400
    time = np.array([t - data['timestamp'][step_start] for t in data['timestamp'][step_start:step_start+duration]])
    response = np.array([s - data['controller']['exp_response'][step_start] for s in data['controller']['exp_response'][step_start:step_start+duration]])

if sys.argv[2]=='motor_1' or sys.argv[2]=='motor_2':
    motor = sys.argv[2]
    running_idx = [i for i, val in enumerate(data[motor]['state']) if val=='running']

    plt.figure()
    plt.plot(np.array(data['timestamp'])[running_idx], np.array(data[motor]['load'])[running_idx], label='step')
    plt.plot(np.array(data['timestamp'])[running_idx], np.array(data[motor]['vel'])[running_idx], label='step response')
    plt.grid()
    plt.legend()

    step_start = 0
    for i in running_idx:
        if data[motor]['load'][i+1] > data[motor]['load'][i]:
            step_start = i
            step_value = data[motor]['load'][i+1] - data[motor]['load'][i]

    duration = 500
    time = np.array([t - data['timestamp'][step_start] for t in data['timestamp'][step_start:step_start+duration]])
    response = np.array([s - data[motor]['vel'][step_start] for s in data[motor]['vel'][step_start:step_start+duration]])

# https://www.ucg.ac.me/skladiste/blog_2146/objava_92847/fajlovi/Astrom.pdf

if sys.argv[3]=='inertial':

    def model(t, K, T1, T2, L):
        t = t - L
        idx = t >= 0
        y = np.zeros_like(t)
        y[idx] = K*(1 + (T2*np.exp(-t[idx]/T2) - T1*np.exp(-t[idx]/T1))/(T1 - T2))
        return y

    def slope(t):
        t = t - L
        idx = t >= 0
        y = np.zeros_like(t)
        y[idx] = K*(-np.exp(-t[idx]/T2) + np.exp(-t[idx]/T1))/(T1 - T2)
        return y

    (K, T1, T2, L), _ = scipy.optimize.curve_fit(model, time, response, p0=[1, 0.1, 0.2, 0.1])

    ideal = model(time, K, T1, T2, L)

    tangent_time = scipy.optimize.minimize_scalar(lambda x: -slope(x), bounds=(0, 10), method='bounded').x
    tangent_a = slope(tangent_time)
    tangent_b = model(tangent_time, K, T1, T2, L) - slope(tangent_time)*tangent_time
    tangent = tangent_a*time + tangent_b
    tangent_idx = (tangent > 0) & (tangent < K)
    tangent_begin = (-tangent_b)/tangent_a
    tangent_end = (-tangent_b + K)/tangent_a

    plt.figure('inertial model')
    plt.plot(time, ideal, color='blue')
    plt.plot(time[tangent_idx], tangent[tangent_idx], linestyle='dashed', color='black')
    plt.vlines(tangent_begin, ymin=0, ymax=K, linestyle='dashed', color='black')
    plt.vlines(tangent_end, ymin=0, ymax=K, linestyle='dashed', color='black')
    plt.plot(time, np.full_like(time, 0), linestyle='dashed', color='black')
    plt.plot(time, np.full_like(time, K), linestyle='dashed', color='black')
    plt.plot(time, response, color='red')
    plt.grid()

    K = K/step_value
    T = tangent_end - tangent_begin
    L = tangent_begin

    print('model')
    print(f'    K = {K:7.3f}')
    print(f'    T = {T:7.3f}')
    print(f'    L = {L:7.3f}')

    T0 = L
    a = K*T0/T
    tau = T0/(T0 + T)

    print('Ziegler-Nichols')
    print(f'          Kp         Ki         Kd')
    Kp = 1/a
    print(f'P   {Kp:10.3f}')
    Kp = 0.9/a
    Ti = 3*T0
    print(f'PI  {Kp:10.3f} {Kp/Ti:10.3f}')
    Kp = 1.2/a
    Ti = 2*T0
    Td = 0.5*T0
    print(f'PID {Kp:10.3f} {Kp/Ti:10.3f} {Kp*Td:10.3f}')

    print('Chien, Hrones, Reswick 0%')
    print(f'          Kp         Ki         Kd')
    Kp = 0.3/a
    print(f'P   {Kp:10.3f}')
    Kp = 0.35/a
    Ti = 1.2*T
    print(f'PI  {Kp:10.3f} {Kp/Ti:10.3f}')
    Kp = 0.6/a
    Ti = T
    Td = 0.5*T0
    print(f'PID {Kp:10.3f} {Kp/Ti:10.3f} {Kp*Td:10.3f}')

    print('Chien, Hrones, Reswick 20%')
    print(f'          Kp         Ki         Kd')
    Kp = 0.7/a
    print(f'P   {Kp:10.3f}')
    Kp = 0.6/a
    Ti = T
    print(f'PI  {Kp:10.3f} {Kp/Ti:10.3f}')
    Kp = 0.95/a
    Ti = 1.4*T
    Td = 0.47*T0
    print(f'PID {Kp:10.3f} {Kp/Ti:10.3f} {Kp*Td:10.3f}')

    print('Cohen-Coon')
    print(f'          Kp         Ki         Kd')
    Kp = (1/a)*(1 + (0.35*tau/(1-tau)))
    print(f'P   {Kp:10.3f}')
    Kp = (0.9/a)*(1 + (0.92*tau/(1 - tau)))
    Ti = ((3.3 - 3*tau)/(1 + 1.2*tau))*T0
    print(f'PI  {Kp:10.3f} {Kp/Ti:10.3f}')
    Kp = (1.24/a)*(1 + (0.13*tau/(1 - tau)))
    Td = ((0.27 - 0.36*tau)/(1 - 0.87*tau))*T0
    print(f'PD  {Kp:10.3f}            {Kp*Td:10.3f}')
    Kp = (1.35/a)*(1 + (0.18*tau/(1 - tau)))
    Ti = ((2.5 - 3*tau)/(1 - 0.39*tau))*T0
    Td = ((0.37 - 0.37*tau)/(1 - 0.81*tau))*T0
    print(f'PID {Kp:10.3f} {Kp/Ti:10.3f} {Kp*Td:10.3f}')

if sys.argv[3]=='integrating':

    def model(t, K, T, L):
        t = t - L
        idx = t >= 0
        y = np.zeros_like(t)
        y[idx] = K*(t[idx] - T*(1 - np.exp(-t[idx]/T)))
        return y

    (K, T, L), _ = scipy.optimize.curve_fit(model, time, response, p0=[1, 0.1, 0.1])

    ideal = model(time, K, T, L)

    tangent = K*time - K*(L+T)
    tangent_idx = tangent > 0
    tangent_begin = L + T

    plt.figure('integrating model')
    plt.plot(time, ideal, color='blue')
    plt.vlines(tangent_begin, ymin=0, ymax=max(response), linestyle='dashed', color='black')
    plt.plot(time[tangent_idx], tangent[tangent_idx], linestyle='dashed', color='black')
    plt.plot(time, np.full_like(time, 0), linestyle='dashed', color='black')
    plt.plot(time, response, color='red')
    plt.grid()

    K = K/step_value
    T = L + T
    Lambda = float(sys.argv[4])

    print('model')
    print(f'    K = {K:7.3f}')
    print(f'    T = {T:7.3f}')

    print('Lambda Tuning')
    print(f'          Kp         Ki')
    Kp = (2*Lambda + T)/(K*((Lambda + T)**2))
    Ti = 2*Lambda + T
    print(f'PI  {Kp:10.3f} {Kp/Ti:10.3f}')

plt.show()
