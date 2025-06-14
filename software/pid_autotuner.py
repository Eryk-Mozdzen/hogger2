import sys
import json
import matplotlib.pyplot as plt
import numpy as np
from collections import defaultdict
import scipy.optimize

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

data_raw = make_recursive_list_dict()

with open(sys.argv[1], 'r') as file:
    for line in file:
        json_obj = json.loads(line)
        telemetry = json_obj.get('telemetry', {})
        if sys.argv[2]!='controller' or telemetry.get('controller', {}).get('started', False):
            collect_values(data_raw, telemetry)

data = convert_to_normal_dict(data_raw)
freq = 100

data['timestamp'] = [(t - data['timestamp'][0])*1e-6 for t in data['timestamp']]

if sys.argv[2]=='controller':
    step_start = np.nonzero(data['controller']['exp_step'])[0][0]
    step_value = max(data['controller']['exp_step'])

    before = 1
    duration = 5
    time = np.array([t - data['timestamp'][step_start] for t in data['timestamp'][step_start-int(before*freq):step_start+int(duration*freq)]])
    step = np.array(data['controller']['exp_step'][step_start-int(before*freq):step_start+int(duration*freq)])
    response = np.array(data['controller']['exp_response'][step_start-int(before*freq):step_start+int(duration*freq)])

if sys.argv[2]=='motor_1' or sys.argv[2]=='motor_2':
    motor = sys.argv[2]
    running_idx = [i for i, val in enumerate(data[motor]['state']) if val=='running']

    step_start = 0
    for i in running_idx:
        if data[motor]['load'][i+1] > data[motor]['load'][i]:
            step_start = i
            step_value = data[motor]['load'][i+1] - data[motor]['load'][i]

    before = 0.5
    duration = 2
    time = np.array([t - data['timestamp'][step_start] for t in data['timestamp'][step_start-int(before*freq):step_start+int(duration*freq)]])
    step = np.array(data[motor]['load'][step_start-int(before*freq):step_start+int(duration*freq)])
    response = np.array(data[motor]['vel'][step_start-int(before*freq):step_start+int(duration*freq)])

if sys.argv[3]=='inertial':

    def model(t, K, T1, T2, L, Y0):
        t = t - L
        idx = t >= 0
        y = np.full_like(t, Y0)
        y[idx] = Y0 + K*(1 + (T2*np.exp(-t[idx]/T2) - T1*np.exp(-t[idx]/T1))/(T1 - T2))
        return y

    (K, T1, T2, L, Y0), _ = scipy.optimize.curve_fit(model, time, response, p0=[1, 0.1, 0.2, 0.1, 0], bounds=[
        [0, 0, 0, 0, -np.inf], np.inf
    ])

    print('model parameters')
    print(f'    K = {K/step_value:f}')
    print(f'   T1 = {T1:f}')
    print(f'   T2 = {T2:f}')
    print(f'    L = {L:f}')
    print(f'   y0 = {Y0:f}')

    ideal = model(time, K, T1, T2, L, Y0)

    tangent_time = L + T1*T2*np.log(T1/T2)/(T1 - T2)
    tangent_a = K*(-np.exp(-(tangent_time - L)/T2) + np.exp(-(tangent_time - L)/T1))/(T1 - T2)
    tangent_b = model(tangent_time, K, T1, T2, L, Y0) - tangent_a*tangent_time
    tangent = tangent_a*time + tangent_b
    tangent_begin = (-tangent_b + Y0)/tangent_a
    tangent_end = (-tangent_b + Y0 + K)/tangent_a

    fig, ax1 = plt.subplots()
    ax2 = ax1.twinx()

    ax1.plot(time, ideal, color='red', label='model', zorder=1)
    ax1.scatter(time, response, color='black', label='response', s=2, zorder=2)
    ax2.plot(time, step, color='blue', label='step')

    ax1.autoscale()
    ax2.autoscale()
    lim1 = ax1.get_ylim()
    lim2 = ax2.get_ylim()

    ax1.plot(time, tangent, linestyle='dashed', color='black', linewidth=0.75, zorder=0)
    ax1.vlines(tangent_end, ymin=-1000, ymax=1000, linestyle='dashed', color='black', linewidth=0.75, zorder=0)
    ax1.plot(time, np.full_like(time, Y0), linestyle='dashed', color='black', linewidth=0.75, zorder=0)
    ax1.plot(time, np.full_like(time, Y0+K), linestyle='dashed', color='black', linewidth=0.75, zorder=0)
    ax2.plot(time, np.full_like(time, step[0]), linestyle='dashed', color='black', linewidth=0.75, zorder=0)

    ax1.grid()
    ax1.set_xlim(-before, duration)
    ax1.set_xlim(-before, duration)
    ax1.set_ylim(lim1)
    ax2.set_ylim(lim2[0] - (lim2[1] - lim2[0]), lim2[1] + (lim2[1] - lim2[0]))

    ax1.set_xlabel('time [s]')
    ax1.set_ylabel('response')
    ax2.set_ylabel('step')

    lines1, labels1 = ax1.get_legend_handles_labels()
    lines2, labels2 = ax2.get_legend_handles_labels()

    ax1.legend(lines2 + lines1, labels2 + labels1, loc='best')

    print('lines')
    print(f'    bottom = {Y0:+7.3f}')
    print(f'       top = {Y0+K:+7.3f}')
    print(f'   tangent = {tangent_a:+7.3f}x{tangent_b:+7.3f}')
    print(f'     begin = {tangent_begin:7.3f}')
    print(f'       end = {tangent_end:7.3f}')

    K = K/step_value
    T = tangent_end - tangent_begin
    L = tangent_begin

    print('geometric parameters')
    print(f'    K = {K:f}')
    print(f'    T = {T:f}')
    print(f'    L = {L:f}')

    T0 = L
    a = K*T0/T
    tau = T0/(T0 + T)

    print('Ziegler-Nichols')
    print(f'          Kp         Ki         Kd')
    Kp = 1/a
    print(f'P   {Kp:10f}')
    Kp = 0.9/a
    Ti = 3*T0
    print(f'PI  {Kp:10f} {Kp/Ti:10f}')
    Kp = 1.2/a
    Ti = 2*T0
    Td = 0.5*T0
    print(f'PID {Kp:10f} {Kp/Ti:10f} {Kp*Td:10f}')

    print('Chien, Hrones, Reswick 0%')
    print(f'          Kp         Ki         Kd')
    Kp = 0.3/a
    print(f'P   {Kp:10f}')
    Kp = 0.35/a
    Ti = 1.2*T
    print(f'PI  {Kp:10f} {Kp/Ti:10f}')
    Kp = 0.6/a
    Ti = T
    Td = 0.5*T0
    print(f'PID {Kp:10f} {Kp/Ti:10f} {Kp*Td:10f}')

    print('Chien, Hrones, Reswick 20%')
    print(f'          Kp         Ki         Kd')
    Kp = 0.7/a
    print(f'P   {Kp:10f}')
    Kp = 0.6/a
    Ti = T
    print(f'PI  {Kp:10f} {Kp/Ti:10f}')
    Kp = 0.95/a
    Ti = 1.4*T
    Td = 0.47*T0
    print(f'PID {Kp:10f} {Kp/Ti:10f} {Kp*Td:10f}')

    print('Cohen-Coon')
    print(f'          Kp         Ki         Kd')
    Kp = (1/a)*(1 + (0.35*tau/(1-tau)))
    print(f'P   {Kp:10f}')
    Kp = (0.9/a)*(1 + (0.92*tau/(1 - tau)))
    Ti = ((3.3 - 3*tau)/(1 + 1.2*tau))*T0
    print(f'PI  {Kp:10f} {Kp/Ti:10f}')
    Kp = (1.24/a)*(1 + (0.13*tau/(1 - tau)))
    Td = ((0.27 - 0.36*tau)/(1 - 0.87*tau))*T0
    print(f'PD  {Kp:10f}            {Kp*Td:10f}')
    Kp = (1.35/a)*(1 + (0.18*tau/(1 - tau)))
    Ti = ((2.5 - 3*tau)/(1 - 0.39*tau))*T0
    Td = ((0.37 - 0.37*tau)/(1 - 0.81*tau))*T0
    print(f'PID {Kp:10f} {Kp/Ti:10f} {Kp*Td:10f}')

if sys.argv[3]=='integrating':

    def model(t, K, T, Y0):
        idx = t >= 0
        y = np.full_like(t, Y0)
        y[idx] = Y0 + K*(t[idx] - T*(1 - np.exp(-t[idx]/T)))
        return y

    (K, T, Y0), _ = scipy.optimize.curve_fit(model, time, response, p0=[1, 0.1, 0], bounds=[
        [0, 0, -np.inf], np.inf
    ])

    ideal = model(time, K, T, Y0)

    tangent = K*time - K*T + Y0

    fig, ax1 = plt.subplots()
    ax2 = ax1.twinx()

    ax1.plot(time, ideal, color='red', label='model', zorder=1)
    ax1.scatter(time, response, color='black', label='response', s=2, zorder=2)
    ax2.plot(time, step, color='blue', label='step')

    ax1.autoscale()
    ax2.autoscale()
    lim1 = ax1.get_ylim()
    lim2 = ax2.get_ylim()

    ax1.plot(time, tangent, linestyle='dashed', color='black', linewidth=0.75, zorder=0)
    ax1.plot(time, np.full_like(time, Y0), linestyle='dashed', color='black', linewidth=0.75, zorder=0)
    ax1.plot(time, tangent, linestyle='dashed', color='black', linewidth=0.75, zorder=0)
    ax2.plot(time, np.full_like(time, step[0]), linestyle='dashed', color='black', linewidth=0.75, zorder=0)

    ax1.grid()
    ax1.set_xlim(-1, 5)
    ax1.set_xlim(-1, 5)
    ax1.set_ylim(lim1)
    ax2.set_ylim(lim2[0] - (lim2[1] - lim2[0]), lim2[1] + (lim2[1] - lim2[0]))

    ax1.set_xlabel('time [s]')
    ax1.set_ylabel('response')
    ax2.set_ylabel('step')

    lines1, labels1 = ax1.get_legend_handles_labels()
    lines2, labels2 = ax2.get_legend_handles_labels()

    ax1.legend(lines2 + lines1, labels2 + labels1, loc='best')

    print('lines')
    print(f'    bottom = {Y0:+7.3f}')
    print(f'   tangent = {K:+7.3f}x{-K*T+Y0:+7.3f}')
    print(f'     begin = {T:+7.3f}')

    K = K/step_value

    print('model')
    print(f'    K = {K:10f}')
    print(f'    T = {T:10f}')

    Lambda = float(sys.argv[4])

    print('Lambda Tuning')
    print(f'          Kp         Ki         Kd')
    Kp = (2*Lambda + T)/(K*((Lambda + T)**2))
    Ti = 2*Lambda + T
    print(f'PI  {Kp:10f} {Kp/Ti:10f}             oscillations')
    Kp = 1/(K*T*Lambda)
    Kd = 1/(K*Lambda)
    print(f'PD  {Kp:10f}            {Kd:10f}  no overshoot')

fig.savefig('pid_autotuner_step_response.svg', bbox_inches='tight', pad_inches=0)

plt.show()
