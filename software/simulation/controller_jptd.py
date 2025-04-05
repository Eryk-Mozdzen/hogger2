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
    [ R*sp.sin(theta), R*sp.cos(theta)*sp.cos(phi1),  R*(sp.sin(theta)*sp.sin(theta1) - sp.cos(theta)*sp.sin(phi1)*sp.cos(theta1)), 0, 0],
    [-R*sp.cos(theta), R*sp.sin(theta)*sp.cos(phi1), -R*(sp.cos(theta)*sp.sin(theta1) + sp.sin(theta)*sp.sin(phi1)*sp.cos(theta1)), 0, 0],
    [0, -R*sp.cos(phi1)/(2*L), R*sp.sin(phi1)*sp.cos(theta1)/(2*L), R*sp.cos(phi2)/(2*L), -R*sp.sin(phi2)*sp.cos(theta2)/(2*L)],
    [1, 0, 0, 0, 0],
    [0, 1, 0, 0, 0],
    [0, 0, 1, 0, 0],
    [1, 0, sp.sin(theta1), 0, -sp.sin(theta2)],
    [0, 0, 0, 1, 0],
    [0, 0, 0, 0, 1],
])

#G = sp.Matrix([
#    [0, 0,  R*(sp.sin(theta)*sp.sin(theta1) - sp.cos(theta)*sp.sin(phi1)*sp.cos(theta1)), 0, 0],
#    [0, 0, -R*(sp.cos(theta)*sp.sin(theta1) + sp.sin(theta)*sp.sin(phi1)*sp.cos(theta1)), 0, 0],
#    [0, 0, R*sp.sin(phi1)*sp.cos(theta1)/(2*L), 0, -R*sp.sin(phi2)*sp.cos(theta2)/(2*L)],
#    [1, 0, 0, 0, 0],
#    [0, 1, 0, 0, 0],
#    [0, 0, 1, 0, 0],
#    [1, 0, sp.sin(theta1), 0, -sp.sin(theta2)],
#    [0, 0, 0, 1, 0],
#    [0, 0, 0, 0, 1],
#])

h = sp.Matrix([
    x,
    y,
    theta,
    psi1,
    psi2,
])

u = sp.Matrix([
    eta[0].diff(t),
    eta[1].diff(t),
    eta[2].diff(t),
    eta[3].diff(t),
    eta[4].diff(t),
])

#u = sp.Matrix([
#    eta[0],
#    eta[1],
#    eta[2].diff(t),
#    eta[3],
#    eta[4].diff(t),
#])

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

def factorize(vec, coeffs):
    A = sp.zeros(len(vec),len(coeffs))
    for i, v in enumerate(vec):
        expr = sp.collect(sp.expand(v), syms=coeffs[:])
        for j, c in enumerate(coeffs):
            A[i, j] = expr.coeff(coeffs[j])
    return A

Kdd = factorize(h2, u)
P = h2 - Kdd*u

Kdd = sp.simplify(Kdd)
P = sp.simplify(P)

sp.pprint(Kdd)
sp.pprint(sp.simplify(Kdd.det()))
sp.pprint(P)

v = sp.Matrix([
    sp.Symbol('v[0]'),
    sp.Symbol('v[1]'),
    sp.Symbol('v[2]'),
    sp.Symbol('v[3]'),
    sp.Symbol('v[4]'),
])

u = Kdd.inv()*(v - P)

u = u.subs([
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
])

source.add_function('linearize_u', 'u', [
    ('v', v),
    ('eta', eta),
    ('q', q)
], u)

K1 = sp.Matrix([[sp.Symbol(f'K1[{5*i + j}]') for j in range(5)] for i in range(5)])
K2 = sp.Matrix([[sp.Symbol(f'K2[{5*i + j}]') for j in range(5)] for i in range(5)])

h = sp.Matrix([
    sp.Symbol('h[0]'),
    sp.Symbol('h[1]'),
    sp.Symbol('h[2]'),
    sp.Symbol('h[3]'),
    sp.Symbol('h[4]'),
])

d_h = sp.Matrix([
    sp.Symbol('d_h[0]'),
    sp.Symbol('d_h[1]'),
    sp.Symbol('d_h[2]'),
    sp.Symbol('d_h[3]'),
    sp.Symbol('d_h[4]'),
])

hd = sp.Matrix([
    sp.Symbol('hd[0]'),
    sp.Symbol('hd[1]'),
    sp.Symbol('hd[2]'),
    sp.Symbol('hd[3]'),
    sp.Symbol('hd[4]'),
])

d_hd = sp.Matrix([
    sp.Symbol('d_hd[0]'),
    sp.Symbol('d_hd[1]'),
    sp.Symbol('d_hd[2]'),
    sp.Symbol('d_hd[3]'),
    sp.Symbol('d_hd[4]'),
])

d2_hd = sp.Matrix([
    sp.Symbol('d2_hd[0]'),
    sp.Symbol('d2_hd[1]'),
    sp.Symbol('d2_hd[2]'),
    sp.Symbol('d2_hd[3]'),
    sp.Symbol('d2_hd[4]'),
])

v = d2_hd - K1*(d_h - d_hd) - K2*(h - hd)

source.add_function('feedback_v', 'v', [
    ('K1', K1),
    ('K2', K2),
    ('h', h),
    ('d_h', d_h),
    ('hd', hd),
    ('d_hd', d_hd),
    ('d2_hd', d2_hd)
], v)

source.add_define('R', 0.05)
source.add_define('L', 0.13)
source.add_define('Omega_1', -300)
source.add_define('Omega_2', +300)

source.generate('../common/control')
