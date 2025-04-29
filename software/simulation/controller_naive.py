import sympy as sp
import c_source_gen

source = c_source_gen.Source('naive')

t = sp.Symbol('t')

x = sp.Function('x')(t)
y = sp.Function('y')(t)
theta = sp.Function('theta')(t)
phi12 = sp.Function('phi_12')(t)
theta12 = sp.Function('theta_12')(t)
omegad = sp.Symbol('omega_d')

R = sp.Symbol('R')
L = sp.Symbol('L')

q = sp.Matrix([
    x,
    y,
    theta,
])

dynamics = sp.Matrix([
    [ R*omegad*sp.cos(theta12)*sp.sin(phi12)],
    [-R*omegad*sp.sin(theta12)],
    [0],
])

result = sp.solve([
    dynamics[0] - x.diff(t),
    dynamics[1] - y.diff(t),
], [phi12, theta12])

phi12d = result[3][0]
theta12d = result[3][1]

xd = sp.Function('x_d')(t)
yd = sp.Function('y_d')(t)

k = sp.Symbol('k')

ex = x - xd
ey = y - yd

ux = xd.diff(t) - k*ex
uy = yd.diff(t) - k*ey

subs = [
    (x.diff(t), ux*sp.cos(-theta) - uy*sp.sin(-theta)),
    (y.diff(t), ux*sp.sin(-theta) + uy*sp.cos(-theta)),
]

gimbal = sp.Matrix([
    phi12d.subs(subs),
    theta12d.subs(subs),
])

gimbal = gimbal.subs([
    (xd.diff(t), sp.Symbol('ref[2]')),
    (yd.diff(t), sp.Symbol('ref[3]')),
    (xd, sp.Symbol('ref[0]')),
    (yd, sp.Symbol('ref[1]')),
    (q[0].diff(t), sp.Symbol('q[3]')),
    (q[1].diff(t), sp.Symbol('q[4]')),
    (q[2].diff(t), sp.Symbol('q[5]')),
    (q[0], sp.Symbol('q[0]')),
    (q[1], sp.Symbol('q[1]')),
    (q[2], sp.Symbol('q[2]')),
    (omegad, sp.Symbol('omega')),
    (R, sp.Symbol('ROBOT_PARAMETER_R')),
    (L, sp.Symbol('ROBOT_PARAMETER_L')),
])

source.add_function(gimbal, 'gimbal', 'feedback(float *gimbal, const float omega, const float *q, const float *ref, const float k)')

source.generate('../common/control')
