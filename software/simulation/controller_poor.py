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
    [ R*sp.sin(theta), R*sp.cos(theta)*sp.cos(phi1), 0],
    [-R*sp.cos(theta), R*sp.sin(theta)*sp.cos(phi1), 0],
    [0, -R*sp.cos(phi1)/(2*L), R*sp.cos(phi2)/(2*L)],
    [1, 0, 0],
    [0, 1, 0],
    [1, 0, 0],
    [0, 0, 1],
])

#G = sp.Matrix([
#    [0, 0, 0],
#    [0, 0, 0],
#    [0, 0, 0],
#    [1, 0, 0],
#    [0, 1, 0],
#    [1, 0, 0],
#    [0, 0, 1],
#])

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

def Lie(base, field, relative_degree=None):
    result = sp.Matrix.zeros(base.shape[0], base.shape[1])
    if relative_degree is None:
        relative_degree = [1 for _ in range(base.shape[0])]
    for row, p in enumerate(relative_degree):
        for col in range(base.shape[1]):
            if p==0:
                result[row, col] = field[row, :]
            elif p==1:
                result[row, col] = field[row, :].jacobian(q)*base
            else:
                result[row, col] =  Lie(base, Lie(base, field[row, :]), p-1)
    return result

#D = sp.Matrix.zeros(h.shape[0], h.shape[0])
#P = sp.Matrix.zeros(h.shape[0], 1)

#p = [2, 2, 2]

#for i in range(h.shape[0]):
#    for j in range(h.shape[0]):
#        D[i, j] = Lie(G[:, j], Lie(F, h[i, :], p[i]-1))
#    P[i] = Lie(F, h[i, :], p[i])

#for i in range(h.shape[0]):
#    for j in range(h.shape[0]):
#        D[i, j] = Lie(G[:, j], h[i, :], p[i]-1)
#    P[i] = Lie(F, h[i, :], p[i]) + Lie(F, Lie(G[:, j]))

#def Lie_derivative(h, f):
#    rows, cols = h.shape
#    L = sp.Matrix.zeros(rows, cols)
#    for i in range(rows):
#        for j in range(cols):
#            for k, var in enumerate(q):
#                L[i,j] += h[i,j].diff(var) * f[k]
#    return L
#
#def Lie(f, h, order=1):
#    if order == 0:
#        return h
#    if f.shape[1] > 1:
#        lie_derivs = []
#        for i in range(f.shape[1]):
#            lie_derivs.append(Lie(f[:, i], Lie_derivative(h, f[:, i]), order-1))
#        return sp.Matrix.hstack(*lie_derivs)
#    L = Lie_derivative(h, f)
#    return Lie(f, L, order-1) if order > 1 else L

def Lie_derivative(h, f):
    rows, cols = h.shape
    L = sp.Matrix.zeros(rows, cols)
    for i in range(rows):
        for j in range(cols):
            for k, var in enumerate(q):
                L[i, j] += h[i, j].diff(var) * f[k]
    return L

def Lie(f, h, order=1, weights=None):
    if order == 0:
        return h

    if f.shape[1] > 1:
        p = f.shape[1]

        if order == 1:
            L = [Lie_derivative(h, f[:, i]) for i in range(p)]
            if weights is None:
                return sp.Matrix.hstack(*L)
            else:
                return sum(weights[i] * L[i] for i in range(p))

        if order == 2 and isinstance(weights, sp.Matrix) and weights.shape == (p, p):
            m, _ = h.shape
            result = sp.Matrix.zeros(m, 1)
            for i in range(p):
                for j in range(p):
                    L1 = Lie_derivative(h, f[:, j])
                    L2 = Lie_derivative(L1, f[:, i])
                    result += weights[i, j] * L2
            return result

        L = [Lie(f[:, i], h, order=order-1, weights=None) for i in range(p)]
        return sp.Matrix.hstack(*L)

    L1 = Lie_derivative(h, f)
    if order == 1:
        return L1
    return Lie(f, L1, order=order - 1, weights=weights)

eta = sp.Matrix([
    sp.Symbol('eta[0]'),
    sp.Symbol('eta[1]'),
    sp.Symbol('eta[2]'),
])

Kdd = Lie(G, h)
#P = Lie(F, h, 2) + Lie(F, Lie(G, h))*eta + Lie(G, Lie(F, h))*eta + (Lie(G, h, 2)*eta).multiply_elementwise(eta)
P = Lie(F, h, 2) + Lie(F, Lie(G, h))*eta + Lie(G, Lie(F, h))*eta + Lie(G, h, 2, eta * eta.T)

#sp.pprint(sp.simplify(Lie(F, h, 2)))
#sp.pprint(sp.simplify(Lie(F, Lie(G, h))*eta))
#sp.pprint(sp.simplify(Lie(G, Lie(F, h))*eta))
#sp.pprint(sp.simplify((Lie(G, h, 2)*eta).multiply_elementwise(eta)))

Kdd = sp.simplify(Kdd)
P = sp.simplify(P)

#sp.pprint(Kdd)
#sp.pprint(sp.simplify(Kdd.det()))
#sp.pprint(P)

v = sp.Matrix([
    sp.Symbol('v[0]'),
    sp.Symbol('v[1]'),
    sp.Symbol('v[2]'),
])

u = Kdd.inv()*(v - P)

u = u.subs([
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
