import numpy as np
import scipy.optimize

R = 0.05
L = 0.13
theta0 = 0
eta_1 = 0
eta_2 = 0
eta_3 = -300
eta_4 = 0
eta_5 = +300

def equations(vars):
    phi_1, theta_1, phi_2, theta_2 = vars

    eq1 = np.cos(theta0 - np.pi/4) - (R*(eta_1*np.sin(theta0) + eta_2*np.cos(phi_1)*np.cos(theta0) - eta_3*(np.sin(phi_1)*np.cos(theta0)*np.cos(theta_1) - np.sin(theta0)*np.sin(theta_1))))
    eq2 = np.sin(theta0 - np.pi/4) - (R*(-eta_1*np.cos(theta0) + eta_2*np.sin(theta0)*np.cos(phi_1) - eta_3*(np.sin(phi_1)*np.sin(theta0)*np.cos(theta_1) + np.sin(theta_1)*np.cos(theta0))))
    eq3 = 0                        - ((1.0/2.0)*R*(-eta_2*np.cos(phi_1) + eta_3*np.sin(phi_1)*np.cos(theta_1) + eta_4*np.cos(phi_2) - eta_5*np.sin(phi_2)*np.cos(theta_2))/L)
    eq4 = 0                        - (eta_1 + eta_3*np.sin(theta_1) - eta_5*np.sin(theta_2))

    return [eq1, eq2, eq3, eq4]

initial_guess = [0, 0, 0, 0]

solution = scipy.optimize.fsolve(equations, initial_guess)

print(f'  phi_1 = {solution[0]:+13.10f}')
print(f'theta_1 = {solution[1]:+13.10f}')
print(f'  phi_2 = {solution[2]:+13.10f}')
print(f'theta_2 = {solution[3]:+13.10f}')
