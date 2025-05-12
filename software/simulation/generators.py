import sympy as sp
import c_source_gen

source = c_source_gen.Source('generators')

t = sp.Symbol('t')

R = sp.Symbol('R')
T = sp.Symbol('T')

w = 2*sp.pi/T
x = R*sp.cos(w*t)
y = R*sp.sin(w*t)
theta = w*t + sp.pi/2

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

hd = hd.subs([
    (R, sp.Symbol('params[0]')),
    (T, sp.Symbol('params[1]')),
])

source.add_function(hd, 'hd', 'circle(float *hd, const float *params, const float t)')

a = sp.Symbol('a')
T = sp.Symbol('T')

w = 2*sp.pi/T
x = a*sp.cos(w*t)*sp.sin(w*t)/((sp.sin(t * w)**2) + 1)
y = a*sp.cos(w*t)/((sp.sin(t * w)**2) + 1)
theta = sp.atan2(y.diff(t), x.diff(t))

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

hd = hd.subs([
    (a, sp.Symbol('params[0]')),
    (T, sp.Symbol('params[1]')),
])

source.add_function(hd, 'hd', 'lemniscate(float *hd, const float *params, const float t)')

v = sp.Symbol('v')

x = v*t
y = 0
theta = 0

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

hd = hd.subs([
    (v, sp.Symbol('params[0]')),
])

source.add_function(hd, 'hd', 'line(float *hd, const float *params, const float t)')

source.generate('../common/control')
