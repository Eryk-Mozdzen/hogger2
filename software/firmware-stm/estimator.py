import sympy as sp

from c_ekf_gen import ekf

dt = sp.Symbol('dt')
hf = sp.Symbol('h_f')
rax, ray = sp.symbols('r_ax r_ay')
rfx, rfy = sp.symbols('r_fx r_fy')
theta0 = sp.Symbol('theta_0')
m0 = sp.Symbol('m_0')
ax0 = sp.Symbol('a_x0')
ay0 = sp.Symbol('a_y0')
wz0 = sp.Symbol('w_z0')

px, py, theta = sp.symbols('px py theta')
vx, vy, vtheta = sp.symbols('vx vy vtheta')
wx, wy, wz = sp.symbols('w_x w_y w_z')
ax, ay, az = sp.symbols('a_x a_y a_z')
mx, my, mz = sp.symbols('m_x m_y m_z')

p = sp.Matrix([px, py])
v = sp.Matrix([vx, vy])
a = sp.Matrix([ax, ay, 0])
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

a_origin = rot3d(theta)*(a - sp.Matrix([ax0, ay0, 0]) - w.cross(w.cross(ra)))

u = sp.Matrix([
    ax,
    ay,
    wz,
])

f = sp.Matrix([
    p[0] + dt*v[0] + 0.5*(dt**2)*a_origin[0],
    p[1] + dt*v[1] + 0.5*(dt**2)*a_origin[1],
    theta + dt*(wz - wz0),
    v[0] + dt*a_origin[0],
    v[1] + dt*a_origin[1],
    wz - wz0,
    m0,
    ax0,
    ay0,
    wz0,
])

h_mag = rot3d(-(theta + theta0))*sp.Matrix([
    sp.cos(m0),
    0,
    sp.sin(m0),
])

Rf = sp.sqrt(rfx**2 + rfy**2)
thetaf = sp.atan2(rfy, rfx)

h_flow = (1/hf)*rot2d(-theta)*sp.Matrix([
    vx - Rf*vtheta*sp.sin(theta + thetaf),
    vy + Rf*vtheta*sp.cos(theta + thetaf),
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
            (ax0, 0, 0.1),
            (ay0, 0, 0.1),
            (wz0, 0, 0.01),
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
        (hf, 0.065),
        (rax, +0.015),
        (ray, -0.16),
        (rfx, -0.09),
        (rfy, -0.13),
        (theta0, 0),
    ],
)

estimator.generate_src('generated')
estimator.generate_docs('generated')
