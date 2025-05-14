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

print(f'step value: {max(data['controller']['exp_step'])}')

plt.plot(data['timestamp'], data['controller']['exp_step'], label='step')
plt.plot(data['timestamp'], data['controller']['exp_response'], label='step response')
plt.grid()

plt.show()
