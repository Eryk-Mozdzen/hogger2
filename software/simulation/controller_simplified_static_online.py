import sympy as sp
import c_source_gen

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

thetau1 = sp.Function('theta_u1')(t)
phiu1 = sp.Function('phi_u1')(t)
thetau2 = sp.Function('theta_u2')(t)
phiu2 = sp.Function('phi_u2')(t)
ru1 = sp.Function('r_u1')(t)
ru2 = sp.Function('r_u2')(t)

R = sp.Symbol('R')
L = sp.Symbol('L')
d = sp.Symbol('d')

qp = sp.Matrix([
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

qu = sp.Matrix([
    x,
    y,
    theta,
    thetau1,
    phiu1,
    thetau2,
    phiu2,
    ru1,
    ru2,
])

G = sp.Matrix([
    [0, sp.cos(theta + thetau1)*ru1, 0, 0, 0],
    [0, sp.sin(theta + thetau1)*ru1, 0, 0, 0],
    [0, sp.csc(thetau2)*sp.sin(thetau1 - thetau2)*ru1/(2*L), 0, 0, 0],
    [1, 0, 0, 0, 0],
    [0, 1, 0, 0, 0],
    [0, 0, 1, 0, 0],
    [0, sp.csc(thetau2)*sp.sin(thetau1)*ru1/ru2, 0, 0, 0],
    [0, 0, 0, 1, 0],
    [0, 0, 0, 0, 1],
])

h = sp.Matrix([
    x + d*sp.cos(thetau1 + theta),
    y + d*sp.sin(thetau1 + theta),
    thetau2,
    ru1,
    ru2,
])

hd = sp.Matrix([
    sp.Function('x_d')(t),
    sp.Function('y_d')(t),
    sp.Function('theta_u2d')(t),
    sp.Function('r_u1d')(t),
    sp.Function('r_u2d')(t),
])

K = sp.Matrix([[sp.Symbol(f'K[{5*i + j}]') for j in range(5)] for i in range(5)])

v = hd.diff(t) - K*(h - hd)

etau = (h.jacobian(qu)*G).inv()*v

#sp.pprint(etau)

def sign(theta_u):
    return sp.Piecewise(
        (+1, (theta_u % sp.pi) < sp.pi/2),
        (+1, (theta_u % sp.pi) > 3*sp.pi/2),
        (-1, True),
    )

etap_integral = sp.Matrix([
    sign(thetau1) * sp.acos(sp.sqrt(ru1**2 - R**2) * sp.sqrt(sp.tan(thetau1)**2 + 1) / sp.sqrt(-R**2 - R**2 * sp.tan(thetau1)**2 + ru1**2 * sp.tan(thetau1)**2)),
    sign(thetau1) * sp.asin(ru1 * sp.tan(thetau1) / (R * sp.sqrt(sp.tan(thetau1)**2 + 1))),
    phiu1,
    sign(thetau2) * sp.asin(ru2 * sp.tan(thetau2) / (R * sp.sqrt(sp.tan(thetau2)**2 + 1))),
    phiu2,
])

etap = etap_integral.diff(t)

#sp.pprint(etap)

qu1 = G*etau

etap = etap.subs({
    (qu[0].diff(t), qu1[0]),
    (qu[1].diff(t), qu1[1]),
    (qu[2].diff(t), qu1[2]),
    (qu[3].diff(t), qu1[3]),
    (qu[4].diff(t), qu1[4]),
    (qu[5].diff(t), qu1[5]),
    (qu[6].diff(t), qu1[6]),
    (qu[7].diff(t), qu1[7]),
    (qu[8].diff(t), qu1[8]),
    (phiu1, psi1),
    (phiu2, psi2),
    (ru1, R * sp.sqrt(sp.cos(phi1)**2 * (sp.sin(theta1)**2 - 1) + 1)),
    (ru2, R * sp.sqrt(sp.cos(phi2)**2 * (sp.sin(theta2)**2 - 1) + 1)),
    (thetau1, sp.atan2(sp.sin(theta1), sp.cos(theta1)*sp.sin(phi1))),
    (thetau2, sp.atan2(sp.sin(theta2), sp.cos(theta2)*sp.sin(phi2))),
})

#sp.pprint(etap)

etap = etap.subs([
    (qp[0], sp.Symbol('q_full[0]')),
    (qp[1], sp.Symbol('q_full[1]')),
    (qp[2], sp.Symbol('q_full[2]')),
    (qp[3], sp.Symbol('q_full[3]')),
    (qp[4], sp.Symbol('q_full[4]')),
    (qp[5], sp.Symbol('q_full[5]')),
    (qp[6], sp.Symbol('q_full[6]')),
    (qp[7], sp.Symbol('q_full[7]')),
    (qp[8], sp.Symbol('q_full[8]')),
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
    (R, sp.Symbol('ROBOT_PARAMETER_R')),
    (L, sp.Symbol('ROBOT_PARAMETER_L')),
    (d, sp.Symbol('ROBOT_PARAMETER_D')),
])

source = c_source_gen.Source('simplified_static_online')

source.add_function(etap, 'eta_full', 'feedback(float *eta_full, const float *K, const float *q_full, const float *hd)')

source.generate('../common/control')
