import sympy as sp

from c_ekf_gen import ekf

dt = sp.Symbol('\\Delta_t')
hf = sp.Symbol('h_f')
rax, ray = sp.symbols('r_{ax} r_{ay}')
rfx, rfy = sp.symbols('r_{fx} r_{fy}')
theta0 = sp.Symbol('\\theta_0')
m0 = sp.Symbol('m_0')

x, y, theta = sp.symbols('x y \\theta')
dx, dy, dtheta = sp.symbols('\\dot{x} \\dot{y} \\dot{\\theta}')
wz, dwz = sp.symbols('\\omega_z \\dot{\\omega}_z')
ax, ay = sp.symbols('a_x a_y')
mx, my, mz = sp.symbols('m_x m_y m_z')

def rot2d(angle):
    return sp.Matrix([
        [sp.cos(angle), -sp.sin(angle)],
        [sp.sin(angle), sp.cos(angle)],
    ])

def rot3d(angle):
    return sp.Matrix([
        [sp.cos(angle), -sp.sin(angle), 0],
        [sp.sin(angle), sp.cos(angle), 0],
        [0, 0, 1],
    ])

A = sp.Matrix([ax, ay, 0])
W = sp.Matrix([0, 0, dtheta])
DW = sp.Matrix([0, 0, dwz])
RA = sp.Matrix([rax, ray, 0])
DDP = rot3d(theta)*(A - DW.cross(RA) - W.cross(W.cross(RA)))

u = sp.Matrix([
    ax,
    ay,
    wz,
    dwz,
])

f = sp.Matrix([
    x + dt*dx + 0.5*(dt**2)*DDP[0],
    y + dt*dy + 0.5*(dt**2)*DDP[1],
    theta + dt*dtheta,
    dx + dt*DDP[0],
    dy + dt*DDP[1],
    wz,
    m0,
])

h_mag = rot3d(-(theta + theta0))*sp.Matrix([
    sp.cos(m0),
    0,
    sp.sin(m0),
])

h_flow = (1/hf)*(rot2d(-theta)*sp.Matrix([
    dx,
    dy,
]) + sp.Matrix([
    -rfy*dtheta,
    +rfx*dtheta,
]))

estimator = ekf.EKF(
    ekf.SystemModel(
        model=f,
        input=u,
        state=[
            (x, 0, 1),
            (y, 0, 1),
            (theta, 0, 1),
            (dx, 0, 1),
            (dy, 0, 1),
            (dtheta, 0, 1),
            (m0, 0, 1),
        ],
    ),
    [
        ekf.MeasurementModel(
            name='mag',
            model=h_mag,
            covariance=1000,
        ),
        ekf.MeasurementModel(
            name='flow',
            model=h_flow,
            covariance=100000,
        ),
    ],
    [
        (dt, 0.001),
        (hf, 0.19),
        (rax, +0.015),
        (ray, -0.16),
        (rfx, 0.14),
        (rfy, -0.13),
        (theta0, 0),
    ],
)

estimator.generate_src('generated')
estimator.generate_docs('generated')
