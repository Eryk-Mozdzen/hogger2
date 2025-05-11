import sympy as sp

from c_ekf_gen import ekf

dt = sp.Symbol('dt')
hf = sp.Symbol('h_f')
rax, ray = sp.symbols('r_ax r_ay')
rfx, rfy = sp.symbols('r_fx r_fy')
theta0 = sp.Symbol('theta0')
m0 = sp.Symbol('m0')

px, py, theta = sp.symbols('px py theta')
vx, vy, vtheta = sp.symbols('vx vy vtheta')
wx, wy, wz = sp.symbols('w_x w_y w_z')
ax, ay, az = sp.symbols('a_x a_y a_z')
mx, my, mz = sp.symbols('m_x m_y m_z')

p = sp.Matrix([px, py])
v = sp.Matrix([vx, vy])
a = sp.Matrix([ax, ay, az])
w = sp.Matrix([0, 0, wz])
m = sp.Matrix([mx, my, mz])
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

a_origin = rot3d(theta)*(a - w.cross(w.cross(ra)))

u = sp.Matrix([
    ax,
    ay,
    wz,
])

f = sp.Matrix([
    p[0] + dt*v[0] + 0.5*(dt**2)*a_origin[0],
    p[1] + dt*v[1] + 0.5*(dt**2)*a_origin[1],
    theta + dt*wz,
    v[0] + dt*a_origin[0],
    v[1] + dt*a_origin[1],
    wz,
    m0,
])

h_mag = rot3d(-(theta + theta0))*sp.Matrix([
    0,
    sp.cos(m0),
    sp.sin(m0),
])

h_flow = (1/hf)*rot2d(-theta)*sp.Matrix([
    vx - rfy*vtheta,
    vy + rfx*vtheta,
])

estimator = ekf.EKF(
    ekf.SystemModel(
        model=f,
        input=u,
        state=[
            (px, 0, 1),
            (py, 0, 1),
            (theta, 0, 1),
            (vx, 0, 1),
            (vy, 0, 1),
            (vtheta, 0, 1),
            (m0, 0, 1),
        ],
    ),
    [
        ekf.MeasurementModel(
            name='magnetometer',
            model=h_mag,
            covariance=10,
        ),
        ekf.MeasurementModel(
            name='flow',
            model=h_flow,
            covariance=10,
        ),
    ],
    [
        (dt, 0.001),
        (hf, 0.065),
        (rax, +0.015),
        (ray, -0.16),
        (rfx, -0.095),
        (rfy, -0.13),
        (theta0, 0),
    ],
)

estimator.generate_src('generated')
estimator.generate_docs('generated')
