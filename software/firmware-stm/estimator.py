import sympy as sp

from c_ekf_gen import ekf

dt = sp.Symbol('dt')
hf = sp.Symbol('h_f')
rax, ray = sp.symbols('r_ax r_ay')
rfx, rfy = sp.symbols('r_fx r_fy')
theta0 = sp.Symbol('theta_0')
m0 = sp.Symbol('m_0')

px, py, theta = sp.symbols('px py theta')
dx, dy, dtheta = sp.symbols('vx vy vtheta')
wz, dwz = sp.symbols('w_z dw_z')
ax, ay = sp.symbols('a_x a_y')
mx, my, mz = sp.symbols('m_x m_y m_z')

a = sp.Matrix([ax, ay, 0])
w = sp.Matrix([0, 0, wz])
dw = sp.Matrix([0, 0, dwz])
ra = sp.Matrix([rax, ray, 0])

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

a_origin = rot3d(theta)*(a - dw.cross(ra) - w.cross(w.cross(ra)))

u = sp.Matrix([
    ax,
    ay,
    wz,
    dwz,
])

f = sp.Matrix([
    px + dt*dx + 0.5*(dt**2)*a_origin[0],
    py + dt*dy + 0.5*(dt**2)*a_origin[1],
    theta + dt*wz,
    dx + dt*a_origin[0],
    dy + dt*a_origin[1],
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
            (px, 0, 1),
            (py, 0, 1),
            (theta, 0, 1),
            (dx, 0, 1),
            (dy, 0, 1),
            (dtheta, 0, 1),
            (m0, 0, 1),
        ],
    ),
    [
        ekf.MeasurementModel(
            name='magnetometer',
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
