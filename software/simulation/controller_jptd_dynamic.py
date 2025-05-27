import sympy as sp
import c_source_gen

source = c_source_gen.Source('jptd_dynamic')

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
    sp.Function('eta_1')(t),
    sp.Function('eta_2')(t),
    sp.Function('eta_3')(t),
    sp.Function('eta_4')(t),
    sp.Function('eta_5')(t),
])

G = sp.Matrix([
    [0, 0,  R*(sp.sin(theta)*sp.sin(theta1) - sp.cos(theta)*sp.sin(phi1)*sp.cos(theta1)), 0, 0],
    [0, 0, -R*(sp.cos(theta)*sp.sin(theta1) + sp.sin(theta)*sp.sin(phi1)*sp.cos(theta1)), 0, 0],
    [0, 0, R*sp.sin(phi1)*sp.cos(theta1)/(2*L), 0, -R*sp.sin(phi2)*sp.cos(theta2)/(2*L)],
    [1, 0, 0, 0, 0],
    [0, 1, 0, 0, 0],
    [0, 0, 1, 0, 0],
    [1, 0, sp.sin(theta1), 0, -sp.sin(theta2)],
    [0, 0, 0, 1, 0],
    [0, 0, 0, 0, 1],
])

h = sp.Matrix([
    x,
    y,
    theta,
    psi1,
    psi2,
])

u = sp.Matrix([
    eta[0],
    eta[1],
    eta[2].diff(t),
    eta[3],
    eta[4].diff(t),
])

q1 = G*eta
q2 = G.diff(t)*eta + G*eta.diff(t)
h2 = h.diff(t, 2).subs([
    (q[0].diff(t, 2), q2[0]),
    (q[1].diff(t, 2), q2[1]),
    (q[2].diff(t, 2), q2[2]),
    (q[3].diff(t, 2), q2[3]),
    (q[4].diff(t, 2), q2[4]),
    (q[5].diff(t, 2), q2[5]),
    (q[6].diff(t, 2), q2[6]),
    (q[7].diff(t, 2), q2[7]),
    (q[8].diff(t, 2), q2[8]),
    (q[0].diff(t), q1[0]),
    (q[1].diff(t), q1[1]),
    (q[2].diff(t), q1[2]),
    (q[3].diff(t), q1[3]),
    (q[4].diff(t), q1[4]),
    (q[5].diff(t), q1[5]),
    (q[6].diff(t), q1[6]),
    (q[7].diff(t), q1[7]),
    (q[8].diff(t), q1[8]),
])

Kdd = h2.jacobian(u)
P = h2 - Kdd*u

Kdd = sp.simplify(Kdd)
P = sp.simplify(P)

sp.pprint(Kdd)
sp.pprint(sp.simplify(Kdd.det()))
sp.pprint(P)

v = sp.Matrix([
    sp.Symbol('v_1'),
    sp.Symbol('v_2'),
    sp.Symbol('v_3'),
    sp.Symbol('v_4'),
    sp.Symbol('v_5'),
])

u = Kdd.inv()*(v - P)

u = u.subs([
    (v[0], sp.Symbol('v[0]')),
    (v[1], sp.Symbol('v[1]')),
    (v[2], sp.Symbol('v[2]')),
    (v[3], sp.Symbol('v[3]')),
    (v[4], sp.Symbol('v[4]')),
    (eta[0], sp.Symbol('eta[0]')),
    (eta[1], sp.Symbol('eta[1]')),
    (eta[2], sp.Symbol('eta[2]')),
    (eta[3], sp.Symbol('eta[3]')),
    (eta[4], sp.Symbol('eta[4]')),
    (q[0], sp.Symbol('q[0]')),
    (q[1], sp.Symbol('q[1]')),
    (q[2], sp.Symbol('q[2]')),
    (q[3], sp.Symbol('q[3]')),
    (q[4], sp.Symbol('q[4]')),
    (q[5], sp.Symbol('q[5]')),
    (q[6], sp.Symbol('q[6]')),
    (q[7], sp.Symbol('q[7]')),
    (q[8], sp.Symbol('q[8]')),
    (R, sp.Symbol('ROBOT_PARAMETER_R')),
    (L, sp.Symbol('ROBOT_PARAMETER_L')),
])

source.add_function(u, 'u', 'linearize_u(float *u, const float *v, const float *eta, const float *q)')

K1 = sp.Matrix([[sp.Symbol(f'K1[{5*i + j}]') for j in range(5)] for i in range(5)])
K2 = sp.Matrix([[sp.Symbol(f'K2[{5*i + j}]') for j in range(5)] for i in range(5)])

h = sp.Matrix([
    sp.Function('h_1')(t),
    sp.Function('h_2')(t),
    sp.Function('h_3')(t),
    sp.Function('h_4')(t),
    sp.Function('h_5')(t),
])

hd = sp.Matrix([
    sp.Function('h_d1')(t),
    sp.Function('h_d2')(t),
    sp.Function('h_d3')(t),
    sp.Function('h_d4')(t),
    sp.Function('h_d5')(t),
])

v = hd.diff(t, 2) - K1*(h.diff(t) - hd.diff(t)) - K2*(h - hd)

v = v.subs([
    (h[0].diff(t), sp.Symbol('h[5]')),
    (h[1].diff(t), sp.Symbol('h[6]')),
    (h[2].diff(t), sp.Symbol('h[7]')),
    (h[3].diff(t), sp.Symbol('h[8]')),
    (h[4].diff(t), sp.Symbol('h[9]')),
    (h[0], sp.Symbol('h[0]')),
    (h[1], sp.Symbol('h[1]')),
    (h[2], sp.Symbol('h[2]')),
    (h[3], sp.Symbol('h[3]')),
    (h[4], sp.Symbol('h[4]')),
    (hd[0].diff(t, 2), sp.Symbol('hd[10]')),
    (hd[1].diff(t, 2), sp.Symbol('hd[11]')),
    (hd[2].diff(t, 2), sp.Symbol('hd[12]')),
    (hd[3].diff(t, 2), sp.Symbol('hd[13]')),
    (hd[4].diff(t, 2), sp.Symbol('hd[14]')),
    (hd[0].diff(t), sp.Symbol('hd[5]')),
    (hd[1].diff(t), sp.Symbol('hd[6]')),
    (hd[2].diff(t), sp.Symbol('hd[7]')),
    (hd[3].diff(t), sp.Symbol('hd[8]')),
    (hd[4].diff(t), sp.Symbol('hd[9]')),
    (hd[0], sp.Symbol('hd[0]')),
    (hd[1], sp.Symbol('hd[1]')),
    (hd[2], sp.Symbol('hd[2]')),
    (hd[3], sp.Symbol('hd[3]')),
    (hd[4], sp.Symbol('hd[4]')),
])

source.add_function(v, 'v', 'feedback_v(float *v, const float *K1, const float *K2, const float *h, const float *hd)')

source.generate('../common/control')
