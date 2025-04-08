"""
This is just a collection of functions for calculating different
things related to the model.
"""

import sys

from sympy import symbols, Matrix, ImmutableMatrix, Eq, simplify, zeros, sin, cos, tan, cot, Symbol, sqrt, atan2, asin, acos, pi, Piecewise, S
from sympy.physics.vector import dynamicsymbols

from sympy_utils import matsym, dynvec, SympyDumpable


t = dynamicsymbols._t


### Symbols ####################################################################
# Single place for defining symbols to avoid unexpected problems with
# inconsistent names.

R = symbols('R')
l = symbols('l')
parameters = {'R': R, 'l': l}

# 5 control inputs
eta = dynvec('eta1:6')

# x, y, theta0, phi1, theta1, psi1, phi2, theta2, psi2 = q
model_full_q__str = 'x y theta_0 phi_1 theta_1 psi_1 phi_2 theta_2 psi_2'
model_full_q = dynvec(model_full_q__str)
model_full_q__exec = '%s = %s' % (', '.join(model_full_q__str.split()),
                                  'dynvec("%s")' % model_full_q__str)

# x, y, theta0, theta_u1, phi_u1, theta_u2, phi_u2, r_u1, r_u2 = q
model_simplified_q__str = 'x y theta_0 theta_u1 phi_u1 theta_u2 phi_u2 r_u1 r_u2'
model_simplified_q = dynvec(model_simplified_q__str)
model_simplified_q__exec = '%s = %s' % (', '.join(model_simplified_q__str.split()),
                                        'dynvec("%s")' % model_simplified_q__str)

### General model class ########################################################

class Model(SympyDumpable):
    """Reprezentation of a kinematic model of a mobile robot

    Models a robot with state vector q(t) and control inputs eta(t).
    The kinematics is defined as:

        d/dt q(t) = G(q) * eta(t)
    """

    @classmethod
    def define(cls, q, A, eta, G):
        i = ImmutableMatrix
        return cls(q=i(q), eta=i(eta), G=i(G).simplify(), A=i(A).simplify())

    def show_as_matrices(self):
        "q' = G * eta"
        return Eq(matsym(r'\dot{q}', self.q), self.G * matsym('eta', self.eta))

    def show_as_equations(self, onevec=True):
        "Show with G * eta multiplied."
        dq = Matrix([q.diff(t) for q in self.q])
        eqs = Eq(dq, self.G * self.eta)
        return eqs

    def nonholonomic_constrains_satisfied(self):
        check = self.A * self.G
        return simplify(check) == zeros(*check.shape)

    def show_pfaff(self):
        pfaff_form = Eq(self.A * matsym(r'\dot{q}', self.q),
                        matsym('0', zeros(self.A.shape[0], 1)))
        return pfaff_form

### Defined models #############################################################

def define_full_1():
    return Model.define(q=model_full_q, A=get_A_full_model(),
                        eta=eta, G=get_G_full_model_1_manual())

def define_full_2():
    return Model.define(q=model_full_q, A=get_A_full_model(),
                        eta=eta, G=get_G_full_model_2_manual())

def define_full_2_JPTD():
    return Model.define(q=model_full_q, A=get_A_full_model(),
                        eta=eta, G=get_G_full_model_2_JPTD())

def define_simplified():
    return Model.define(q=model_simplified_q, A=get_A_simplified_model(),
                        eta=eta, G=get_G_simplified_model())

### full model from Paweł Joniak Bachelor's thesis #############################

def get_A_full_model():
    q = model_full_q
    x, y, theta0, phi1, theta1, psi1, phi2, theta2, psi2 = q

    s, c = sin, cos

    A = Matrix([
        [-c(theta0), -s(theta0), 0, 0, R*c(phi1), -R*c(theta1)*s(phi1), 0, 0, 0],
        [s(theta0), -c(theta0), 0, -R, 0, -R*s(theta1), 0, 0, 0],
        [-c(theta0), -s(theta0), -2*l, 0, 0, 0, 0, R*c(phi2), -R*c(theta2)*s(phi2)],
        [s(theta0), -c(theta0), 0, 0, 0, 0, -R, 0, -R*s(theta2)],
    ])
    return A


def get_generators_full_model():
    q = model_full_q
    x, y, theta0, phi1, theta1, psi1, phi2, theta2, psi2 = q

    s, c = sin, cos

    generators = [
        [R*c(phi1)*c(theta0), R*s(theta0)*c(phi1), -R*c(phi1)/(2*l), 0, 1, 0, 0, 0, 0],
        [-R*s(phi1)*c(theta0)*c(theta1), -R*s(phi1)*s(theta0)*c(theta1), R*s(phi1)*c(theta1)/(2*l), -s(theta1), 0, 1, 0, 0, 0],
        [R*s(theta0), -R*c(theta0), 0, 1, 0, 0, 1, 0, 0],
        [0, 0, R*c(phi2)/(2*l), 0, 0, 0, 0, 1, 0],
        [R*s(theta0)*s(theta2), -R*s(theta2)*c(theta0), -R*s(phi2)*c(theta2)/(2*l), s(theta2), 0, 0, 0, 0, 1],
    ]

    generators = [Matrix(g) for g in generators]

    return generators

def get_G_full_model_1_Joniak():
    # direct control over deviations[?] and one spinning
    q = model_full_q
    x, y, theta0, phi1, theta1, psi1, phi2, theta2, psi2 = q

    # controls matrix
    s, c = sin, cos
    R_ = R / (2*l)
    cc = s(theta0)*s(theta1) - c(theta0)*c(theta2)*s(phi2)
    sc = c(theta0)*s(theta1) + s(theta0)*c(theta2)*s(phi2)
    d = c(theta1)*s(phi1) - cot(theta2)*s(theta1)*s(phi2)
    G = Matrix([
        [R*s(theta0), R*c(theta0)*c(phi1), R*cc, 0, 0],
        [R*c(theta0), R*s(theta0)*c(phi1), -R*sc, 0, 0],
        [-R_*cot(theta2)*s(phi2), -R_*c(phi1), R_*d, R_*cot(theta2)*s(phi2), R_*c(phi2)],
        [1, 0, 0, 0, 0],
        [0, 1, 0, 0, 0],
        [0, 0, 1, 0, 0],
        [0, 0, 0, 1, 0],
        [0, 0, 0, 0, 1],
        [1/s(theta2), 0, s(theta1)/s(theta2), -1/s(theta2), 0],
    ])

    return G

def get_G_full_model_2_Joniak():
    # direct control over both spinnings and three deviations[?]
    q = model_full_q
    x, y, theta0, phi1, theta1, psi1, phi2, theta2, psi2 = q

    # controls matrix
    s, c = sin, cos
    R_ = R / (2*l)
    cc = s(theta0)*s(theta1) - c(theta0)*c(theta1)*s(phi1)  # different than in 1!
    sc = c(theta0)*s(theta1) - s(theta0)*c(theta1)*s(phi1)  # different than in 1!
    # d = c(theta1)*s(phi1) - cot(theta2)*s(theta1)*s(phi2)
    G = Matrix([
        [R*s(theta0), R*c(theta0)*c(phi1), R*cc, 0, 0],
        [-R*c(theta0), R*s(theta0)*c(phi1), -R*sc, 0, 0],
        [0, -R_*c(phi1), -R_*c(theta1)*s(phi1), -R_*c(phi2), -R_*c(theta2)*s(phi2)],
        [1, 0, 0, 0, 0],
        [0, 1, 0, 0, 0],
        [0, 0, 1, 0, 0],
        [1, 0, s(theta1), 0, -s(theta1)],
        [0, 0, 0, 1, 0],
        [0, 0, 0, 0, 1],
    ])

    return G


### full model done manually ###################################################

def get_G_full_model_1_manual():
    # direct control over deviations[?] and one spinning
    q = model_full_q
    x, y, theta0, phi1, theta1, psi1, phi2, theta2, psi2 = q

    generators = get_generators_full_model()
    g1, g2, g3, g4, g5 = generators

    s, c = sin, cos
    g25 = g2 + s(theta1)/s(theta2)*g5
    g35 = g3 - g5/s(theta2)
    G = Matrix([_g.T for _g in [g3 - g35, g1, g25, g35, g4]]).T

    return G

def get_G_full_model_2_manual():
    # direct control over both spinnings and three deviations[?]
    q = model_full_q
    x, y, theta0, phi1, theta1, psi1, phi2, theta2, psi2 = q

    generators = get_generators_full_model()
    g1, g2, g3, g4, g5 = generators

    s, c = sin, cos
    g25 = g2 + g3*s(theta1)
    g35 = g5 - g3*s(theta2)
    G = Matrix([_g.T for _g in [g3, g1, g25, g4, g35]]).T

    return G

### JPTD algorithm on full model ###############################################

def get_G_full_model_2_JPTD():
    #  # direct control over both spinnings and three deviations[?]
    #  q = model_full_q
    #  x, y, theta0, phi1, theta1, psi1, phi2, theta2, psi2 = q

    #  # controls matrix
    #  s, c = sin, cos
    #  R_ = R / (2*l)
    #  cc = s(theta0)*s(theta1) - c(theta0)*c(theta1)*s(phi1)
    #  sc = c(theta0)*s(theta1) - s(theta0)*c(theta1)*s(phi1)
    #  G = Matrix([
    #      [0, 0, R*cc, 0, 0],
    #      [0, 0, -R*sc, 0, 0],
    #      [0, 0, -R_*c(theta1)*s(phi1), 0, -R_*c(theta2)*s(phi2)],
    #      [1, 0, 0, 0, 0],
    #      [0, 1, 0, 0, 0],
    #      [0, 0, 1, 0, 0],
    #      [1, 0, s(theta1), 0, -s(theta2)],
    #      [0, 0, 0, 1, 0],
    #      [0, 0, 0, 0, 1],
    #  ])

    G = get_G_full_model_2_manual()
    G[0, 0] *= 0;
    G[0, 1] *= 0;
    G[1, 0] *= 0;
    G[1, 1] *= 0;
    G[2, 1] *= 0;
    G[2, 3] *= 0;

    return G


### Simplified model ###########################################################

def get_A_simplified_model(subs_from_thesis=False):
    q = model_simplified_q
    x, y, theta0, theta_u1, phi_u1, theta_u2, phi_u2, r_u1, r_u2 = q

    s, c = sin, cos
    s0u1 = s(theta0 + theta_u1)
    c0u1 = c(theta0 + theta_u1)
    s0u2 = s(theta0 + theta_u2)
    c0u2 = c(theta0 + theta_u2)
    su2 = s(theta_u2)
    cu2 = c(theta_u2)

    A = Matrix([
        [s0u1, -c0u1, 0, 0, 0, 0, 0, 0, 0],
        [c0u1, s0u1, 0, 0, -r_u1, 0, 0, 0, 0],
        [s0u2, -c0u2, 2*l*su2, 0, 0, 0, 0, 0, 0],
        [c0u2, s0u2, 2*l*cu2, 0, 0, 0, -r_u2, 0, 0],
    ])

    # substitutions used in Bachelor thesis
    if subs_from_thesis:
        A = A.subs({
            s(theta0 + theta_u1): Symbol('s_{0u_1}'),
            c(theta0 + theta_u1): Symbol('c_{0u_1}'),
            s(theta0 + theta_u2): Symbol('s_{0u_2}'),
            c(theta0 + theta_u2): Symbol('c_{0u_2}'),
            s(theta_u2): Symbol('s_{u_2}'),
            c(theta_u2): Symbol('c_{u_2}'),
        })

    return A

def get_G_simplified_model():
    q = model_simplified_q
    x, y, theta0, theta_u1, phi_u1, theta_u2, phi_u2, r_u1, r_u2 = q

    s, c = sin, cos
    G = Matrix([
        [0, c(theta0+theta_u1)*r_u1, 0, 0, 0],
        [0, s(theta0+theta_u1)*r_u1, 0, 0, 0],
        [0, s(theta_u1-theta_u2)*r_u1/(2*l*sin(theta_u2)), 0, 0, 0],
        [1, 0, 0, 0, 0],
        [0, 1, 0, 0, 0],
        [0, 0, 1, 0, 0],
        [0, s(theta_u1)*r_u1/(s(theta_u2)*r_u2), 0, 0, 0],
        [0, 0, 0, 1, 0],
        [0, 0, 0, 0, 1],
    ])

    return G

def convert_simplified_to_full(expr):
    """
    Replaces variables of simplified model in the expression with variables of full model.
    This in fact is the function that has to be used when converting data of the full model
    to simplified model, which is unintuitive but correct.
    """
    print('[Warning] convert_simplified_to_full has not been verified', file=sys.stderr)

    q = model_full_q
    x, y, theta0, phi1, theta1, psi1, phi2, theta2, psi2 = q
    q = model_simplified_q
    x, y, theta0, theta_u1, phi_u1, theta_u2, phi_u2, r_u1, r_u2 = q

    s, c = sin, cos
    subs = {
        phi_u1: psi1,
        phi_u2: psi2,
        r_u1: R * sqrt(c(phi1)**2 * (s(theta1)**2 - 1) + 1),
        r_u2: R * sqrt(c(phi2)**2 * (s(theta2)**2 - 1) + 1),
        theta_u1: atan2(s(theta1), c(theta1)*s(phi1)),
        theta_u2: atan2(s(theta2), c(theta2)*s(phi2)),
    }

    return expr.subs(subs)

def convert_full_to_simplified(expr):
    """
    Replaces variables of full model in the expression with variables of simplified model.
    This has to be used when implementing conversion from simplified to full (!).
    The variables with acos/asin (phi1, phi2, theta1, theta2) have sign computed using
    Piecewise depending on theta_ui: "+" when theta_u is in (-pi/2, pi/2), "-" elsewhere.
    """
    print('[Warning] convert_full_to_simplified has not been verified', file=sys.stderr)

    q = model_full_q
    x, y, theta0, phi1, theta1, psi1, phi2, theta2, psi2 = q
    q = model_simplified_q
    x, y, theta0, theta_u1, phi_u1, theta_u2, phi_u2, r_u1, r_u2 = q

    def sign(theta_u):
        # we use modulo, so theta will be
        theta_mod = (theta_u % pi)
        return Piecewise(
            # + for theta from (-pi/2, pi/2) --> (0, pi/2) OR (3/2pi, 2pi)
            (S(1), theta_mod < pi/2),
            (S(1), theta_mod > 3*pi/2),
            (-S(1), True)
        )

    s, c = sin, cos
    subs = {
        psi1: phi_u1,
        psi2: phi_u2,
        phi1: sign(theta_u1) * acos(sqrt(r_u1**2 - R**2) * sqrt(tan(theta_u1)**2 + 1) / sqrt(-R**2 - R**2 * tan(theta_u1)**2 + r_u1**2 * tan(theta_u1)**2)),
        phi2: sign(theta_u2) * acos(sqrt(r_u2**2 - R**2) * sqrt(tan(theta_u2)**2 + 1) / sqrt(-R**2 - R**2 * tan(theta_u2)**2 + r_u2**2 * tan(theta_u2)**2)),
        theta1: sign(theta_u1) * asin(r_u1 * tan(theta_u1) / (R * sqrt(tan(theta_u1)**2 + 1))),
        theta2: sign(theta_u2) * asin(r_u2 * tan(theta_u2) / (R * sqrt(tan(theta_u2)**2 + 1))),
    }

    return expr.subs(subs)

