import sys

K = float(sys.argv[1])
T = float(sys.argv[2])
L = float(sys.argv[3])

print('Lambda Tuning')
print(f'          Kp         Ki         Kd')
Kp = (2*L + T)/(K*((L + T)**2))
Ti = 2*L + T
print(f'PI  {Kp:10.3f} {Kp/Ti:10.3f}')
