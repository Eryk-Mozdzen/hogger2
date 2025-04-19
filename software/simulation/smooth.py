import sympy as sp

t = sp.Symbol('t')
T = sp.Symbol('T')

u = t/T

a = sp.symbols('a0:8')
f = sum(a[i]*u**i for i in range(8))

conds = [
    0 - f.subs(t, 0),                #    f(0) = 0
    0 - sp.diff(f, t).subs(t, 0),    #   f'(0) = 0
    0 - sp.diff(f, t, 2).subs(t, 0), #  f''(0) = 0
    0 - sp.diff(f, t, 3).subs(t, 0), # f'''(0) = 0
    1 - f.subs(t, T),                #    f(T) = 1
    0 - sp.diff(f, t).subs(t, T),    #   f'(T) = 0
    0 - sp.diff(f, t, 2).subs(t, T), #  f''(T) = 0
    0 - sp.diff(f, t, 3).subs(t, T)  # f'''(T) = 0
]

solution = sp.solve(conds, a)

s = f.subs(solution)

s0 = sp.simplify(s)
s1 = sp.simplify(s.diff(t))
s2 = sp.simplify(s.diff(t, 2))
s3 = sp.simplify(s.diff(t, 3))

print(sp.ccode(s0))
print(sp.ccode(s1))
print(sp.ccode(s2))
print(sp.ccode(s3))
