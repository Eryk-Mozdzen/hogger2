import sympy as sp
import c_source_gen

source = c_source_gen.Source('jptd_dynamic_1d')

t = sp.Symbol('t')

x = sp.Function('x')(t)
y = sp.Function('y')(t)
theta = sp.Function('theta')(t)
phi1 = sp.Function('phi_1')(t)
phi2 = sp.Function('phi_2')(t)
psi = sp.Function('psi')(t)

R = sp.Symbol('R')
L = sp.Symbol('L')
d = sp.Symbol('d')

q = sp.Matrix([
    x,
    y,
    theta,
    phi1,
    phi2,
    psi,
])

eta = sp.Matrix([
    sp.Function('eta_1')(t),
    sp.Function('eta_2')(t),
    sp.Function('eta_3')(t),
])

G = sp.Matrix([
    [0, 0, -R*sp.cos(theta)*sp.sin(phi1)],
    [0, 0, -R*sp.sin(theta)*sp.sin(phi1)],
    [0, 0, R*sp.sin(phi1)/(2*L) + R*sp.sin(phi2)/(2*L)],
    [1, 0, 0],
    [0, 1, 0],
    [0, 0, 1],
])

h = sp.Matrix([
    x + d*sp.cos(theta),
    y + d*sp.sin(theta),
    psi,
])

u = sp.Matrix([
    eta[0],
    eta[1],
    eta[2].diff(t),
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
    (q[0].diff(t), q1[0]),
    (q[1].diff(t), q1[1]),
    (q[2].diff(t), q1[2]),
    (q[3].diff(t), q1[3]),
    (q[4].diff(t), q1[4]),
    (q[5].diff(t), q1[5]),
])

Kdd = h2.jacobian(u)
P = h2 - Kdd*u

#assert sp.simplify(h2)==sp.simplify(Kdd*u + P), 'Dynamic linearization error: wrong calculation of Kdd and P!'

sp.pprint(sp.simplify(Kdd))
sp.pprint(sp.simplify(Kdd.det()))
sp.pprint(sp.simplify(P))

v = sp.Matrix([
    sp.Symbol('v_1'),
    sp.Symbol('v_2'),
    sp.Symbol('v_3'),
])

Kdd = sp.simplify(Kdd)
P = sp.simplify(P)

u = Kdd.inv()*(v - P)

#u = sp.simplify(u)

u = u.subs([
    (v[0], sp.Symbol('v[0]')),
    (v[1], sp.Symbol('v[1]')),
    (v[2], sp.Symbol('v[2]')),
    (eta[0], sp.Symbol('eta[0]')),
    (eta[1], sp.Symbol('eta[1]')),
    (eta[2], sp.Symbol('eta[2]')),
    (q[0], sp.Symbol('q[0]')),
    (q[1], sp.Symbol('q[1]')),
    (q[2], sp.Symbol('q[2]')),
    (q[3], sp.Symbol('q[3]')),
    (q[4], sp.Symbol('q[4]')),
    (q[5], sp.Symbol('q[5]')),
    (R, sp.Symbol('ROBOT_PARAMETER_R')),
    (L, sp.Symbol('ROBOT_PARAMETER_L')),
    (d, sp.Symbol('ROBOT_PARAMETER_D')),
])

source.add_function(u, 'u', 'linearize_u(float *u, const float *v, const float *eta, const float *q)')

K1 = sp.Matrix([[sp.Symbol(f'K1[{3*i + j}]') for j in range(3)] for i in range(3)])
K2 = sp.Matrix([[sp.Symbol(f'K2[{3*i + j}]') for j in range(3)] for i in range(3)])

h = sp.Matrix([
    sp.Function('h_1')(t),
    sp.Function('h_2')(t),
    sp.Function('h_3')(t),
])

hd = sp.Matrix([
    sp.Function('h_d1')(t),
    sp.Function('h_d2')(t),
    sp.Function('h_d3')(t),
])

v = hd.diff(t, 2) - K1*(h.diff(t) - hd.diff(t)) - K2*(h - hd)

v = v.subs([
    (h[0].diff(t), sp.Symbol('h[3]')),
    (h[1].diff(t), sp.Symbol('h[4]')),
    (h[2].diff(t), sp.Symbol('h[5]')),
    (h[0], sp.Symbol('h[0]')),
    (h[1], sp.Symbol('h[1]')),
    (h[2], sp.Symbol('h[2]')),
    (hd[0].diff(t, 2), sp.Symbol('hd[6]')),
    (hd[1].diff(t, 2), sp.Symbol('hd[7]')),
    (hd[2].diff(t, 2), sp.Symbol('hd[8]')),
    (hd[0].diff(t), sp.Symbol('hd[3]')),
    (hd[1].diff(t), sp.Symbol('hd[4]')),
    (hd[2].diff(t), sp.Symbol('hd[5]')),
    (hd[0], sp.Symbol('hd[0]')),
    (hd[1], sp.Symbol('hd[1]')),
    (hd[2], sp.Symbol('hd[2]')),
])

source.add_function(v, 'v', 'feedback_v(float *v, const float *K1, const float *K2, const float *h, const float *hd)')

source.generate('../common/control')
