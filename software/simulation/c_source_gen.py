import os
import sympy as sp
import sympy.codegen.ast
import datetime

class Function:
    def __init__(self, name, output, inputs, expr):
        self.name = name
        self.output = output
        self.inputs = inputs
        self.expr = expr

    def header(self, file):
        file.write(f'void {self.name}(float *{self.output}')
        for name, symbol in self.inputs:
            file.write(f', const float *{name}')
        file.write(');\n')

    def source(self, file):
        file.write(f'void {self.name}(float *{self.output}')
        for name, symbol in self.inputs:
            file.write(f', const float *{name}')
        file.write(') {\n')

        functions = {
            'Pow': [
                (lambda base, exponent: exponent==2, lambda base, exponent: '(%s)*(%s)' % (base, base)),
                (lambda base, exponent: exponent!=2, lambda base, exponent: 'powf(%s, %s)' % (base, exponent))
            ],
        }

        aliases = {
            sympy.codegen.ast.real: sympy.codegen.ast.float32,
        }

        for i, expr in enumerate(list(self.expr)):
            file.write(f'    {self.output}[{i}] = {sp.ccode(expr, user_functions=functions, type_aliases=aliases)};\n')

        file.write('}\n')

class Source:
    def __init__(self, name):
        self.name = name
        self.functions = []
        self.defines = []

    def add_function(self, name, output, inputs, expr):
        self.functions.append(Function(self.name + '_' + name, output, inputs, expr))

    def add_define(self, name, value):
        self.defines.append((name, value))

    def generate(self, path):
        os.makedirs(path, exist_ok=True)

        here = os.path.dirname(__file__)
        header_path = here + '/' + path + '/' + self.name + '.h'
        source_path = here + '/' + path + '/' + self.name + '.c'

        with open(header_path, 'w') as file:
            file.write(Source.__header())
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
            file.write('#include <math.h>\n')
            file.write('\n')
            for name, value in self.defines:
                file.write(f'#define {name} {str(value)}\n')
            file.write('\n')
            for func in self.functions:
                func.source(file)
                file.write('\n')

    def __header(comment='//'):
        text = \
        '{comment} auto-generated\n' \
        '{comment} {time} {date}\n' \
        '\n'

        return text.format(
            comment=comment,
            time=datetime.datetime.now().strftime('%H:%M:%S'),
            date=datetime.datetime.now().strftime('%d-%m-%Y'),
        )
