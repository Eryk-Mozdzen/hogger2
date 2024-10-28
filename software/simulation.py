import sympy as sp

t = sp.Symbol('t')

x = sp.Function('x')(t)
y = sp.Function('y')(t)
theta = sp.Function('theta')(t)
phi1 = sp.Function('phi_1')(t)
theta1 = sp.Function('theta_1')(t)
psi1 = sp.Function('psi_1')(t)
phi2 = sp.Function('phi_2')(t)
theta2 = sp.Function('theta_2')(t)
psi2 = sp.Function('psi_2')(t)

R = sp.Symbol('R')
L = sp.Symbol('L')

eta1 = sp.Symbol('eta_1')
eta2 = sp.Symbol('eta_2')
eta3 = sp.Symbol('eta_3')
eta4 = sp.Symbol('eta_4')
eta5 = sp.Symbol('eta_5')

xd = sp.Function('x_d')(t)
yd = sp.Function('y_d')(t)
thetad = sp.Function('theta_d')(t)
psi1d = sp.Function('psi_1d')(t)
psi2d = sp.Function('psi_2d')(t)

q = sp.Matrix([
    x,
    y,
    theta,
    phi1,
    theta1,
    psi1,
    phi2,
    theta2,
    psi2,
])

eta = sp.Matrix([
    eta1,
    eta2,
    eta3,
    eta4,
    eta5,
])

f = sp.Matrix([
    R*( sp.sin(theta)*eta1 + sp.cos(theta)*sp.cos(phi1)*eta2 + (sp.sin(theta)*sp.sin(theta1) - sp.cos(theta)*sp.cos(theta1)*sp.sin(phi1))*eta3),
    R*(-sp.cos(theta)*eta1 + sp.sin(theta)*sp.cos(phi1)*eta2 - (sp.cos(theta)*sp.sin(theta1) + sp.sin(theta)*sp.cos(theta1)*sp.sin(phi1))*eta3),
    R*(-sp.cos(phi1)*eta2 - sp.cos(theta1)*sp.cos(phi1)*eta3 + sp.cos(phi2)*eta4 - sp.cos(theta2)*sp.sin(phi2)*eta5)/(2*L),
    eta1,
    eta2,
    eta3,
    eta1 + sp.sin(theta1)*eta3 - sp.sin(theta2)*eta5,
    eta4,
    eta5,
])

h = sp.Matrix([
    x,
    y,
    theta,
    psi1,
    psi2,
])

hd = sp.Matrix([
    xd,
    yd,
    thetad,
    psi1d,
    psi2d,
])

K = sp.Symbol('K')

G = f.jacobian(eta)
u = sp.diff(hd, t) - K*(h - hd)
eta_u = (h.jacobian(q)*G).inv()*u

dynamics = [(sp.diff(q[i], t), f[i]) for i in range(q.shape[0])]
eta_sol = sp.solve([eta_u[i].subs(dynamics) - eta[i] for i in range(eta.shape[0])], [eta1, eta2, eta3, eta4, eta5])

feedback = [
    (eta1, eta_sol[eta1]),
    (eta2, eta_sol[eta2]),
    (eta3, eta_sol[eta3]),
    (eta4, eta_sol[eta4]),
    (eta5, eta_sol[eta5]),

    (xd, sp.cos(2*sp.pi*0.1*t)),
    (yd, sp.sin(2*sp.pi*0.1*t)),
    (thetad, sp.pi/2 + 2*sp.pi*0.1*t),
    (psi1d, 1000),
    (psi2d, 1000),
]

parameters = [
    (R, 0.05),
    (L, 0.13),
    (K, 1),
]

fnumeric = sp.lambdify([x, y, theta, phi1, theta1, psi1, phi2, theta2, psi2, t], sp.simplify(f.subs(feedback).subs(parameters)))

import scipy
import numpy as np

def dynamics(t, q):
    x, y, theta, phi1, theta1, psi1, phi2, theta2, psi2 = q
    dxdt = fnumeric(x, y, theta, phi1, theta1, psi1, phi2, theta2, psi2, t)
    return np.array(dxdt).flatten()

solution = scipy.integrate.solve_ivp(
    fun=dynamics,
    t_span=(0, 10),
    y0=[0, 0, np.pi/2, 0, 0, 0, 0, 0, 0],
    method='RK45',
    rtol=1e-6,
    atol=1e-9,
    dense_output=True,
)

import matplotlib.pyplot as plt

plt.plot(solution.y[0], solution.y[1], color='red')
plt.title('Hogger^2 trajectory tracking')
plt.xlabel('x [m]')
plt.xlabel('y [m]')
plt.grid()

time = np.linspace(solution.t[0], solution.t[-1], 25)
indices = [np.abs(solution.t - t).argmin() for t in time]
for i in indices:
    plt.arrow(
        solution.y[0][i],
        solution.y[1][i],
        0.25*np.cos(solution.y[2][i]),
        0.25*np.sin(solution.y[2][i]),
        head_width=0.02,
        head_length=0.05,
        fc='black',
        ec='black',
    )

plt.show()
