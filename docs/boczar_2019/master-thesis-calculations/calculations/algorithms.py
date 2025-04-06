"""
This file defines different control algorithms implementations on Model
"""

from sympy import Eq, solve, latex, Symbol, Matrix, zeros, MatrixSymbol
from sympy.physics.vector import dynamicsymbols

from sympy_utils import SympyDumpable, matsym


t = dynamicsymbols._t


class StaticLinearization(SympyDumpable):
    """Static state feedback linearization algorithm

    This class calculates and holds the static feedback algorithm
    parameters for the given model and output function h(q).

    Static linearization is an algorithm that allows us to transform the
    system with input eta(t) and output q(t) to a new system with input:

        u(t) = d/dt h(q) = dh(q)/dq * G(q) * eta(t)                (1)

    This allows us to control the system with a simple control algorithm
    defined as (P regulator):

        u(t) = d/dt hd(q) - K * (h(q) - hd(q))

    The actual control input for the object can be calculated from (1) as:

        eta(t) = (dh(q)/dt * G(q))^(-1) * u(t)

    The algorithm is defined by the decoupling matrix:

        D(q) = dh(q)/dt * G(q)

    If D(q) is singular, then the algorithm cannot be applied.
    """

    @classmethod
    def compute(cls, model, h):
        si = lambda m: m.as_immutable().simplify() if m else None
        D = cls._compute(model, h)
        D_inv = None
        try:
            D_inv = D.inv()
        except:
            pass
        return cls(h=si(h), D=si(D), D_inv=si(D_inv))

    @classmethod
    def _compute(cls, model, h):
        """
        To compute D(q) we have to differentiate h(q):

            d/dt h(q) = dh(q)/dt * d/dt q

        and substitute:

            d/dt q(t) = G(q) * eta(t)

        Then we get:

            d/dt h(q) = dh(q)/dt * G(q) * eta(t) = D(q) * eta(t)
        """
        q, G = model.q, model.G
        D = h.jacobian(q) * G
        return D

    def show(self, model):
        G = matsym('G', model.G)
        dh_dq = matsym(r'\frac{\delta h}{\delta q}', self.h.jacobian(model.q))
        det = self.D.det().simplify()
        eqs = []
        if hasattr(self, 'D_inv') and self.D_inv:
            eqs.append(Eq((dh_dq * G).inv(), self.D_inv))
        eqs.append(Eq(Symbol(r'\det \left(%s\right)' % latex(dh_dq * G)), det))
        try:
            eqs.append(solve(Eq(det, 0)))
        except:
            pass
        return eqs

    #  def _compute_controls(cls, model, h, D_inv):
    #      # symbol for desired trajectory
    #      hd = MatrixSymbol('h_d', h.shape[0])
    #      # gain matrix
    #      K = MatrixSymbol('K', h.shape[0], h.shape[0])
    #
    #      # control algorithm
    #      u = hd.diff(t) - K * (h - hd)
    #      eta = D_inv * u
    #
    #      return eta
    #
    #  def _compute_ode(cls, model, h, eta):
    #      # simulation object's differential equation
    #      dq = model.G * eta


class DynamicLinearization(SympyDumpable):
    """Dynamic state feedback linearization algorithm

    This class calculates and holds the dynamic feedback algorithm
    parameters for the given model and output function h(q).

    Dynamic linearization transforms the system with input eta(t) and
    output q(t) into a new system with input:

        v(t) = d^2/dt^2 h(q) = Kdd(q) * u(t) + P(q, eta)

    Where u(t) is a new intermediate control input which elements are
    either from d/dt eta(t) or eta(t) when d/dt eta(t) does not appear.
    In gerenal u(t) adds at least one derivative of elements of eta(t)
    so that its state must be remebered and the derivatives have to be
    integrated to get the actual value of input to the model.

    This allows to control the system using PD[?or PI?] regulator algorithm:

        v(t) = d^2/dt^2 hd(q) - K1 * (d/dt h(q) - d/dt hd(q))
                              - K2 * (     h(q) -      hd(q))

    The input u(t) can be calculated as:

        u(t) = Kdd(q)^(-1) * (v(t) - P(q, eta))

    and so Kdd(q) must be invertible.
    """

    @classmethod
    def compute(cls, model, h):
        si = lambda m: m.as_immutable().simplify() if m else None
        Kdd, P, u = cls._compute(model, h)
        Kdd_inv = None
        try:
            Kdd_inv = Kdd.inv()
        except:
            pass
        return cls(h=si(h), u=si(u), P=si(P), Kdd=si(Kdd), Kdd_inv=si(Kdd_inv))

    @classmethod
    def _compute(cls, model, h):
        # try computing for whole u(t) being derivatives of eta(t)
        u = model.eta.diff(t)
        Kdd, P = cls._compute_for(model, h, u)

        # when the determinant is zero we have to find columns that caused it
        # and replace those derivatives d/dt eta(t)[i] with just eta(t)[i]
        if Kdd.det().simplify().is_zero:
            # TODO: this could probably be implemented in a more elegant way
            #       I've done it literally the way I was doing it manually
            def is_zero_col(i):
                col = Kdd[:, i].as_immutable().simplify()
                return col == zeros(Kdd.shape[0], 1)

#             not_derivatives = [is_zero_col(i) for i in range(Kdd.shape[1])]
            not_derivatives = map(is_zero_col, range(Kdd.shape[1]))

            def take_ui(i, take_derivative):
                eta_i = model.eta[i]
                return eta_i.diff() if take_derivative else eta_i

            derivatives = map(lambda x: not x, not_derivatives)
            u = map(lambda args: take_ui(*args), enumerate(derivatives))
            u = Matrix(list(u))

            Kdd, P = cls._compute_for(model, h, u)

        return Kdd, P, u

    @classmethod
    def _compute_for(cls, model, h, u):
        q, G, eta = model.q, model.G, model.eta
        subs_dq = {dq: ge for (dq, ge) in zip(q.diff(t), G * eta)}

        # differentiate and substitute 2 times
        dh = h.diff(t).subs(subs_dq)
        d2h = dh.diff(t).subs(subs_dq)

        # get Kdd matrix back by computing Jacobian
        Kdd = d2h.jacobian(u)
        P = d2h - Kdd * u

        # check to be sure that we did it correstly
        error = d2h - (Kdd * u + P)
        assert error.as_immutable().simplify().is_zero, \
            'Dynamic linearization error: wrong calculation of Kdd and P!'

        return Kdd, P

    def show(self, model):
        u = matsym(r'u', model.eta)
        det = self.Kdd.det().simplify()

        eqs = []
        eqs.append(Eq(matsym(r'\ddot h', self.h), self.Kdd * u + self.P))
        eqs.append(Eq(u, self.u))
        eqs.append(Eq(Symbol(r'\det \left(K_{dd}\right)'), det))
        try:
            eqs.append(solve(Eq(det, 0)))
        except:
            pass

        return eqs
