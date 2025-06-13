import sympy as sp
import c_source_gen

source = c_source_gen.Source('generators')

t = sp.Symbol('t')

def add_trajectory(name, x, y, theta):
    p = sp.Matrix([
        x,
        y,
        theta,
    ])

    hd = sp.Matrix([
        p,
        p.diff(t, 1),
        p.diff(t, 2),
        p.diff(t, 3),
    ])

    source.add_function(hd, 'hd', f'{name}(float *hd, const float *params, const float t)')

R = sp.Symbol('params[0]')
T = sp.Symbol('params[1]')
w = 2*sp.pi/T
x = R*sp.cos(w*t)
y = R*sp.sin(w*t)
theta = w*t + sp.pi/2

add_trajectory('circle', x, y, theta)

a = sp.Symbol('params[0]')
T = sp.Symbol('params[1]')
w = 2*sp.pi/T
x = a*sp.cos(w*t)*sp.sin(w*t)/((sp.sin(t * w)**2) + 1)
y = a*sp.cos(w*t)/((sp.sin(t * w)**2) + 1)
theta = sp.atan2(y.diff(t), x.diff(t))

add_trajectory('lemniscate', x, y, theta)

v = sp.Symbol('params[0]')

add_trajectory('line', v*t, 0, 0)

a = sp.Symbol('params[0]')
T = sp.Symbol('params[1]')
w = 4*sp.pi/T
x = a*sp.sin(w*(t/2))
y = a*sp.cos(w*(t - sp.pi/4))
theta = sp.atan2(y.diff(t), x.diff(t))

add_trajectory('lissajous', x, y, theta)

source.generate('../common/control')
