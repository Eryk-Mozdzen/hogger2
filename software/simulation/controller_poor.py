import sympy as sp
import c_source_gen

source = c_source_gen.Source('jptd_dynamic')

t = sp.Symbol('t')

x = sp.Function('x')(t)
y = sp.Function('y')(t)
theta = sp.Function('theta')(t)
phi1 = sp.Function('phi_1')(t)
theta1 = sp.Function('theta_1')(t)
phi2 = sp.Function('phi_2')(t)
theta2 = sp.Function('theta_2')(t)

R = sp.Symbol('R')
L = sp.Symbol('L')
W1 = sp.Symbol('Omega_1')
W2 = sp.Symbol('Omega_2')

xd = sp.Function('x_d')(t)
yd = sp.Function('y_d')(t)
thetad = sp.Function('theta_d')(t)

q = sp.Matrix([
    x,
    y,
    theta,
    phi1,
    theta1,
    phi2,
    theta2,
])

eta = sp.Matrix([
    phi1.diff(t),
    theta1.diff(t),
    theta2.diff(t),
])

G = sp.Matrix([
    [0, 0, 0],
    [0, 0, 0],
    [0, 0, 0],
    [1, 0, 0],
    [0, 1, 0],
    [1, 0, 0],
    [0, 0, 1],
])

F = sp.Matrix([
    [ W1*R*(sp.sin(theta)*sp.sin(theta1) - sp.cos(theta)*sp.sin(phi1)*sp.cos(theta1))],
    [-W1*R*(sp.cos(theta)*sp.sin(theta1) + sp.sin(theta)*sp.sin(phi1)*sp.cos(theta1))],
    [W1*R*sp.sin(phi1)*sp.cos(theta1)/(2*L) - W2*R*sp.sin(phi2)*sp.cos(theta2)/(2*L)],
    [0],
    [0],
    [W1*sp.sin(theta1) - W2*sp.sin(theta2)],
    [0],
])

h = sp.Matrix([
    x,
    y,
    theta,
])

def Lie(field1, field2, order=1):
    if order==0:
        return field2
    if order==1:
        return field2.jacobian(q)*field1
    return Lie(field1, Lie(field1, field2), order-1)

D = sp.Matrix.zeros(h.shape[0], h.shape[0])
P = sp.Matrix.zeros(h.shape[0], 1)

p = [2, 2, 2]

for i in range(h.shape[0]):
    for j in range(h.shape[0]):
        D[i, j] = Lie(G[:, j], Lie(F, h[i, :], p[i]-1))
    P[i] = Lie(F, h[i, :], p[i])

#sp.pprint(sp.simplify(D))
#sp.pprint(sp.simplify(D.det()))
#sp.pprint(sp.simplify(P))

v = sp.Matrix([
    sp.Symbol('v[0]'),
    sp.Symbol('v[1]'),
    sp.Symbol('v[2]'),
])

u = D.inv()*(v - P)

u = u.subs([
    (eta[0], sp.Symbol('eta[0]')),
    (eta[1], sp.Symbol('eta[1]')),
    (eta[2], sp.Symbol('eta[2]')),

    (q[0], sp.Symbol('q[0]')),
    (q[1], sp.Symbol('q[1]')),
    (q[2], sp.Symbol('q[2]')),
    (q[3], sp.Symbol('q[3]')),
    (q[4], sp.Symbol('q[4]')),
    (q[5], sp.Symbol('q[5]')),
    (q[6], sp.Symbol('q[6]')),
])

#u = sp.simplify(u)

source.add_function('linearize_u', 'u', [
    ('v', v),
    ('eta', eta),
    ('q', q)
], u)

source.generate('../common/control')

K1 = sp.Matrix([[sp.Symbol(f'K1[{3*i + j}]') for j in range(3)] for i in range(3)])
K2 = sp.Matrix([[sp.Symbol(f'K2[{3*i + j}]') for j in range(3)] for i in range(3)])

h = sp.Matrix([
    sp.Symbol('h[0]'),
    sp.Symbol('h[1]'),
    sp.Symbol('h[2]'),
])

d_h = sp.Matrix([
    sp.Symbol('d_h[0]'),
    sp.Symbol('d_h[1]'),
    sp.Symbol('d_h[2]'),
])

hd = sp.Matrix([
    sp.Symbol('hd[0]'),
    sp.Symbol('hd[1]'),
    sp.Symbol('hd[2]'),
])

d_hd = sp.Matrix([
    sp.Symbol('d_hd[0]'),
    sp.Symbol('d_hd[1]'),
    sp.Symbol('d_hd[2]'),
])

d2_hd = sp.Matrix([
    sp.Symbol('d2_hd[0]'),
    sp.Symbol('d2_hd[1]'),
    sp.Symbol('d2_hd[2]'),
])

v = d2_hd - K1*(d_h - d_hd) - K2*(h - hd)

v = v.subs([
    (h[0], sp.Symbol('h[0]')),
    (h[1], sp.Symbol('h[1]')),
    (h[2], sp.Symbol('h[2]')),
])

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
