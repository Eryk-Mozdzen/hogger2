import sys

K = float(sys.argv[1])
T = float(sys.argv[2])
T0 = float(sys.argv[3])

a = K*T0/T
tau = T0/(T0 + T)

print(f'tau = {tau:.3f}')

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
