import numpy as np
import pandas as pd
import matplotlib.pyplot as plt

file = pd.read_csv('output.csv')

plt.figure('trajectory')
plt.plot(file['xd'].values, file['yd'].values, label='xy reference', color='black', linestyle='dashed')
plt.plot(file['x'].values, file['y'].values, label='xy', color='red')
plt.xlabel('x [m]')
plt.ylabel('y [m]')
plt.gca().set_aspect('equal')
plt.grid()
plt.legend()

plt.figure('control signals')
plt.plot(file['t'].values, np.degrees(file['phi1'].values), label='$\\phi_1$')
plt.plot(file['t'].values, np.degrees(file['phi2'].values), label='$\\phi_2$')
plt.xlabel('t [s]')
plt.ylabel('gimbal angles [deg]')
plt.legend()
plt.grid()

plt.figure('reference tracking error')
plt.plot(file['t'].values, file['x'].values - file['xd'].values, label='$e_x$')
plt.plot(file['t'].values, file['y'].values - file['yd'].values, label='$e_y$')
plt.xlabel('t [s]')
plt.ylabel('error [m]')
plt.grid()
plt.legend()

plt.show()
