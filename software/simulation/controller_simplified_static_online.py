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
    [0, sp.sin(thetau1 - thetau2)*ru1/(2*L*sp.sin(thetau2)), 0, 0, 0],
    [1, 0, 0, 0, 0],
    [0, 1, 0, 0, 0],
    [0, 0, 1, 0, 0],
    [0, sp.sin(thetau1)*ru1/(sp.sin(thetau2)*ru2), 0, 0, 0],
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

#v = sp.Matrix([
#    sp.Symbol('v_1'),
#    sp.Symbol('v_2'),
#    sp.Symbol('v_3'),
#    sp.Symbol('v_4'),
#    sp.Symbol('v_5'),
#])

etau = (h.jacobian(qu)*G).inv()*v

#sp.pprint(etau)

def sign(theta_u):
    theta_mod = (theta_u % (2*sp.pi))
    return sp.Piecewise(
        (+1, theta_mod < sp.pi/2),
        (+1, theta_mod > 3*sp.pi/2),
        (-1, True),
    )

g = sp.Matrix([
    x,
    y,
    theta,
    sign(thetau1) * sp.acos(sp.sqrt(R**2 - ru1**2) * sp.sqrt(sp.tan(thetau1)**2 + 1) / sp.sqrt(R**2 + R**2 * sp.tan(thetau1)**2 - ru1**2 * sp.tan(thetau1)**2)),
    sign(thetau1) * sp.asin(ru1 * sp.tan(thetau1) / (R * sp.sqrt(sp.tan(thetau1)**2 + 1))),
    phiu1,
    sign(thetau2) * sp.acos(sp.sqrt(R**2 - ru2**2) * sp.sqrt(sp.tan(thetau2)**2 + 1) / sp.sqrt(R**2 + R**2 * sp.tan(thetau2)**2 - ru2**2 * sp.tan(thetau2)**2)),
    sign(thetau2) * sp.asin(ru2 * sp.tan(thetau2) / (R * sp.sqrt(sp.tan(thetau2)**2 + 1))),
    phiu2,
])

f = {
    (x, x),
    (y, y),
    (theta, theta),
    (thetau1, sp.atan2(sp.sin(theta1), sp.cos(theta1)*sp.sin(phi1))),
    (phiu1, psi1),
    (thetau2, sp.atan2(sp.sin(theta2), sp.cos(theta2)*sp.sin(phi2))),
    (phiu2, psi2),
    (ru1, R * sp.sqrt(sp.cos(phi1)**2 * (sp.sin(theta1)**2 - 1) + 1)),
    (ru2, R * sp.sqrt(sp.cos(phi2)**2 * (sp.sin(theta2)**2 - 1) + 1)),
}

qp1 = (g.jacobian(qu)*G*etau).subs(f)

#print(qp1.shape)

etap = sp.Matrix([
    qp1[3],
    qp1[4],
    qp1[5],
    qp1[7],
    qp1[8],
])

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

source.add_function(etap, 'eta_full', 'calculate(float *eta_full, const float *K, const float *q_full, const float *hd)')

source.generate('../common/control')
