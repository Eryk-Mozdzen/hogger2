import json
import functools
from copy import copy

import sympy
from sympy import *
from sympy.physics.vector import dynamicsymbols
from sympy.printing.latex import LatexPrinter
from sympy.core.function import UndefinedFunction, AppliedUndef

# we use time...all the time
t = dynamicsymbols._t


class ShortLatexPrinter(LatexPrinter):
    """
    Printer for simplified printing in LaTeX mode.
    The printer allows to:
    - discard time arguement of functions that depend on time only, e.g. only "q" instead of "q(t)"
    - print trigonometric functions of single argument with the argument as a subscript
    - remap function names for some trigonometric functions
    """
    # configuration of the printer class
    # these must be class variables, as the class is being instantiated each time when printing
    ENABLED = True  # enable/disable all
    NO_TIME = True  # do not print time argument
    SHORT_TRIG = True  # use s(x) instead of sin(x), etc.
    SHORT_TRIG_FUNCTIONS_MAP = {
        'sin': 's',
        'cos': 'c',
        'tan': r'\operatorname{tan}',
        'cot': r'\operatorname{cot}',
        #  'tg': r'\operatorname{tg}',
        #  'ctg': r'\operatorname{ctg}',
        #  'sec': r'\operatorname{sec}',
        #  'csc': r'\operatorname{csc}',
    }
    REMAP_FUNCIONS = True
    # TODO: conflicts with SympyDumpable.save
    REMAP_FUNCIONS_MAP = {
        #  'tan': 'tg',
        #  'cot': 'ctg',
    }
    DOTTED_DERIVATIVE = True  # use dot instead of d/dt

    def _print_Function(self, expr, exp=None):
        if ShortLatexPrinter.ENABLED:
            func = expr.func.__name__
            if hasattr(self, '_print_' + func) and \
                    not isinstance(expr.func, UndefinedFunction):
                return getattr(self, '_print_' + func)(expr, exp)
            if ShortLatexPrinter.NO_TIME and \
                    len(expr.args) == 1 and expr.args[0] == dynamicsymbols._t:
                # this should recursively call this function
                #  expr = copy(expr)
                #  expr.args = []
                #  return self.doprint(expr.func)

                if exp is not None:
                    return r'%s^{%s}' % (self._hprint_Function(func), exp)
                else:
                    return r'%s' % (self._hprint_Function(func))
                # expr = expr.func  # TODO: maybe this would work?
            if ShortLatexPrinter.REMAP_FUNCIONS:
                if expr.func.__name__ in ShortLatexPrinter.REMAP_FUNCIONS_MAP.keys():
                    expr = copy(expr)
                    expr.func.__name__ = ShortLatexPrinter.REMAP_FUNCIONS_MAP[expr.func.__name__]
            if ShortLatexPrinter.SHORT_TRIG:
                # only for funcions with one argument
                if len(expr.args) == 1:
                    arg = expr.args[0]
                    if expr.func.__name__ in ShortLatexPrinter.SHORT_TRIG_FUNCTIONS_MAP.keys():
                        name = ShortLatexPrinter.SHORT_TRIG_FUNCTIONS_MAP[expr.func.__name__]

                        if exp is not None:
                            return '%s^{%s}_{%s}' % (name, exp, self.doprint(arg))
                        else:
                            return '%s_{%s}' % (name, self.doprint(arg))
        # fallback to defaults
        return super()._print_Function(expr, exp)

    # taken from https://docs.sympy.org/latest/_modules/sympy/physics/vector/printing.html
    def _print_Derivative(self, der_expr):
        if not ShortLatexPrinter.ENABLED:
            # fallback to defaults
            return super()._print_Derivative(der_expr)

        # make sure it is in the right form
        der_expr = der_expr.doit()
        if not isinstance(der_expr, Derivative):
            return r"\left(%s\right)" % self.doprint(der_expr)

        # check if expr is a dynamicsymbol
        t = dynamicsymbols._t
        expr = der_expr.expr
        red = expr.atoms(AppliedUndef)
        syms = der_expr.variables
        test1 = not all([True for i in red if i.free_symbols == {t}])
        test2 = not all([(t == i) for i in syms])
        if test1 or test2:
            return LatexPrinter().doprint(der_expr)

        # done checking
        dots = len(syms)
        base = self._print_Function(expr)
        base_split = base.split('_', 1)
        base = base_split[0]
        if dots == 1:
            base = r"\dot{%s}" % base
        elif dots == 2:
            base = r"\ddot{%s}" % base
        elif dots == 3:
            base = r"\dddot{%s}" % base
        elif dots == 4:
            base = r"\ddddot{%s}" % base
        else: # Fallback to standard printing
            return LatexPrinter().doprint(der_expr)
        if len(base_split) is not 1:
            base += '_' + base_split[1]
        return base


# source code copied from sympy.printing.latex()
def short_latex(expr, fold_frac_powers=False, fold_func_brackets=False,
                fold_short_frac=None, inv_trig_style="abbreviated",
                itex=False, ln_notation=False, long_frac_ratio=None,
                mat_delim="[", mat_str=None, mode="plain", mul_symbol=None,
                order=None, symbol_names=None):

    if symbol_names is None:
        symbol_names = {}

    settings = {
        'fold_frac_powers': fold_frac_powers,
        'fold_func_brackets': fold_func_brackets,
        'fold_short_frac': fold_short_frac,
        'inv_trig_style': inv_trig_style,
        'itex': itex,
        'ln_notation': ln_notation,
        'long_frac_ratio': long_frac_ratio,
        'mat_delim': mat_delim,
        'mat_str': mat_str,
        'mode': mode,
        'mul_symbol': mul_symbol,
        'order': order,
        'symbol_names': symbol_names,
    }

    return ShortLatexPrinter(settings).doprint(expr)


def matsym(symbol_name, like_matrix):
    "Create a MatrixSymbol to be used to represent 'like_matrix' for printing in equations"
    return MatrixSymbol(symbol_name, *like_matrix.shape)

def vec(*args):
    return Matrix([*args])

def ivec(*args):
    return ImmutableMatrix([*args])

def dynvec(*args, **kwargs):
    "Create a vector of dynamicsymbols as Matrix"
    return Matrix([*dynamicsymbols(*args, **kwargs)])


def cache_matrix_args(func):
    """
    Cache for class-methods(!) with all the arguments being SymPy matrix expressions.
    It compares arguments using sympy arithmetics.
    It has unlimited storage and stores values in a list (no hashing) - this is not
    designed for handling many different input arguments, this function is well suited
    for just a few input arguments .
    Works for methods without arguments (i.e. def foo(self): ...).
    """

    # list of tuples (arguments, return_values)
    cache = []

    @functools.wraps(func)
    def wrapper(self, *args):
        # check each cache entry linearily (unefficient for many entries)
        for c_args, c_returns in cache:
            if len(args) == len(c_args):
                # compare all arguments, if any is different we break
                all_same = True
                for arg, c_arg in zip(args, c_args):
                    is_sympy = lambda x: isinstance(x, tuple(sympy.core.all_classes))
                    # if we deal with sympy types
                    if is_sympy(arg) and is_sympy(c_arg):
                        difference = arg - c_arg
                        # simplify the expression (mutable matrices are simplified
                        #  differently, so convert to Immutable before simplifying)
                        difference = ImmutableMatrix(difference).simplify()
                        if not difference.is_zero:
                            all_same = False
                            break
                    # not sympy types (e.g function takes None)
                    else:
                        if arg != c_arg:
                            all_same = False
                            break
                # if all the arguments were equal we can just return from cache
                if all_same:
                    return c_returns
        # args not found in cache, call the function and save return value
        returns = func(self, *args)
        cache.append((args, returns))
        return returns

    wrapper.cache = cache
    return wrapper


class SympyDumpable:
    """
    Because computing some things each time takes too much time we want these
    to be saved and loaded easily. As SymPy Function objects do not support
    pickling, we want to save them using srepr() and then load using eval().

    This class uses dirty metaprogramming to dump all its attributes to a file
    and then to load them by using eval(). Members should be SymPy expressions
    or simple types (avoid referencing other classes).

    Subclasses should NOT define constructor (or be VERY careful). They should
    be constructed using classmethod/staticmethod instead and pass attributes
    by keyword arguments.
    """

    def __init__(self, **kwargs):
        for name, value in kwargs.items():
            setattr(self, name, value)

    def srepr(self):
        representation = {name: srepr(value) for name, value in vars(self).items()
                          if isinstance(value, tuple(sympy.core.all_classes))}
        return json.dumps(representation)

    #  def save(self, file_path):
    #      representation = {name: srepr(value) for name, value in vars(self).items()}
    #      with open(file_path, 'w') as file:
    #          file.write(json.dumps(representation))

    @classmethod
    def loads(cls, srepr):
        representation = json.loads(srepr)
        kwargs = {name: eval(s_repr) for name, s_repr in representation.items()}
        return cls(**kwargs)

    #  @classmethod
    #  def load(cls, file_path):
    #      with open(file_path) as file:
    #          return cls.loads(file.read())
    #          representation = json.loads(file.read())
    #      kwargs = {name: eval(s_repr) for name, s_repr in representation.items()}
    #      return cls(**kwargs)

    @staticmethod
    def save(file_path, objects_by_name):
        "e.g. objects_by_name = {'model': my_model}"
        json_dict = {}
        for name, obj in objects_by_name.items():
            json_dict[name] = obj.srepr()
        with open(file_path, 'w') as file:
            file.write(json.dumps(json_dict, indent=4, sort_keys=True, allow_nan=False))

    @staticmethod
    def load(file_path, classes_with_names):
        """
        e.g. classes_with_names = {'model': Model, 'model2': Model}
        returns a dictionary with classes substituted by objects
        """
        with open(file_path) as file:
            json_dict = json.loads(file.read())
        objects = {}
        for name, cls in classes_with_names:
            objects[name] = cls.loads(json_dict[name])
        return objects

