import os
import sympy as sp
import sympy.codegen.ast
import datetime

class Function:
    def __init__(self, expr, output, name):
        self.expr = expr
        self.output = output
        self.name = name

    def header(self, file):
        file.write(f'void {self.name};\n')

    def source(self, file):
        file.write('void ' + self.name + ' {\n')

        functions = {
            'Pow': [
                (lambda base, exponent: exponent==2, lambda base, exponent: '((%s)*(%s))' % (base, base)),
                (lambda base, exponent: exponent==-1, lambda base, exponent: '(1.f/(%s))' % (base)),
                (lambda base, exponent: True, lambda base, exponent: 'powf(%s, %s)' % (base, exponent))
            ],
        }

        aliases = {
            sympy.codegen.ast.real: sympy.codegen.ast.float32,
        }

        def decl(lhs, rhs):
            file.write(f'    const float {lhs} = {sp.ccode(rhs, user_functions=functions, type_aliases=aliases)};\n')

        cse_subs, cse_reduced = sp.cse([self.expr], optimizations='basic')
        cse_reduced, = cse_reduced

        for lhs, rhs in cse_subs:
            decl(lhs, rhs)
        file.write('\n')

        self.expr = cse_reduced

        for i, expr in enumerate(list(self.expr)):
            file.write(f'    {self.output}[{i}] = {sp.ccode(expr, user_functions=functions, type_aliases=aliases)};\n')

        file.write('}\n')

class Source:
    def __init__(self, prefix):
        self.prefix = prefix
        self.functions = []

    def add_function(self, expr, output, name):
        self.functions.append(Function(expr, output, self.prefix + '_' + name))

    def generate(self, path):
        os.makedirs(path, exist_ok=True)

        here = os.path.dirname(__file__)
        header_path = here + '/' + path + '/' + self.prefix + '.h'
        source_path = here + '/' + path + '/' + self.prefix + '.c'

        with open(header_path, 'w') as file:
            file.write(Source.__header())
            file.write('\n')
            file.write('#ifdef __cplusplus\n')
            file.write('extern "C" {\n')
            file.write('#endif\n')
            file.write('\n')
            for func in self.functions:
                func.header(file)
            file.write('\n')
            file.write('#ifdef __cplusplus\n')
            file.write('}\n')
            file.write('#endif\n')

        with open(source_path, 'w') as file:
            file.write(Source.__header())
            file.write('\n')
            file.write('#include <math.h>\n')
            file.write('\n')
            file.write('#include "control/robot_parameters.h"\n')
            file.write('\n')
            for func in self.functions:
                func.source(file)
                file.write('\n')

    def __header(comment='//'):
        text = \
        '{comment} auto-generated\n' \
        '{comment} {time} {date}\n'

        return text.format(
            comment=comment,
            time=datetime.datetime.now().strftime('%H:%M:%S'),
            date=datetime.datetime.now().strftime('%d-%m-%Y'),
        )
