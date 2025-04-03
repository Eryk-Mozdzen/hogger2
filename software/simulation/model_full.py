import sympy as sp

t = sp.Symbol('t')

x = sp.Symbol('x')
y = sp.Symbol('y')
theta = sp.Symbol('theta')
phi1 = sp.Symbol('phi_1')
theta1 = sp.Symbol('theta_1')
phi2 = sp.Symbol('phi_2')
theta2 = sp.Symbol('theta_2')

R = sp.Symbol('R')
L = sp.Symbol('L')
W1 = sp.Symbol('Omega_1')
W2 = sp.Symbol('Omega_2')

q = sp.Matrix([
    x,
    y,
    theta,
    phi1,
    theta1,
    phi2,
    theta2,
])

eta = sp.Matrix([
    sp.Symbol('eta_1'),
    sp.Symbol('eta_2'),
    sp.Symbol('eta_3'),
])

G = sp.Matrix([
    [ R*sp.sin(theta), R*sp.cos(theta)*sp.cos(phi1), 0],
    [-R*sp.cos(theta), R*sp.sin(theta)*sp.cos(phi1), 0],
    [0, -R*sp.cos(phi1)/(2*L), R*sp.cos(phi2)/(2*L)],
    [1, 0, 0],
    [0, 1, 0],
    [1, 0, 0],
    [0, 0, 1],
])

F = sp.Matrix([
    [ W1*R*(sp.sin(theta)*sp.sin(theta1) - sp.cos(theta)*sp.sin(phi1)*sp.cos(theta1))],
    [-W1*R*(sp.cos(theta)*sp.sin(theta1) + sp.sin(theta)*sp.sin(phi1)*sp.cos(theta1))],
    [W1*R*sp.sin(phi1)*sp.cos(theta1)/(2*L) - W2*R*sp.sin(phi2)*sp.cos(theta2)/(2*L)],
    [0],
    [0],
    [W1*sp.sin(theta1) - W2*sp.sin(theta2)],
    [0],
])

dynamics = G*eta + F

dynamics = sp.simplify(dynamics)

sp.pprint(dynamics)

sp.pprint(dynamics.free_symbols)

parameters = {
    R: 0.05,
    L: 0.13,
    W1: -300,
    W2: +300,
}

import os

here = os.path.dirname(__file__)

with open(f'{here}/src/Model.hpp', 'w') as file:
    file.write(
'''#pragma once

#include <drake/systems/framework/leaf_system.h>

class Model : public drake::systems::LeafSystem<double> {
    void DoCalcTimeDerivatives(const drake::systems::Context<double> &context, drake::systems::ContinuousState<double> *derivatives) const;
    void eval(const drake::systems::Context<double> &context, drake::systems::BasicVector<double> *output) const;

public:
'''
    )
    for param, value in parameters.items():
        file.write(f'    static constexpr double {sp.ccode(param)} = {value:e};\n')
    file.write(
'''
    Model();

    const drake::systems::InputPort<double> & get_control_input_port() const;
    const drake::systems::OutputPort<double> & get_state_output_port() const;
};
'''
    )

with open(f'{here}/src/Model.cpp', 'w') as file:
    file.write(
'''#include "Model.hpp"

Model::Model() {
'''
    )
    file.write('    this->DeclareContinuousState({0, 0, 0, 0, 0, 0.1, 0.1});\n')
    file.write('    this->DeclareVectorInputPort("eta", ' + str(eta.shape[0]) + ');\n')
    file.write('    this->DeclareVectorOutputPort("q", ' + str(2*q.shape[0]) + ', &Model::eval, {this->all_state_ticket()});\n')
    file.write(
'''}

void Model::DoCalcTimeDerivatives(const drake::systems::Context<double> &context, drake::systems::ContinuousState<double> *derivatives) const {
'''
    )
    file.write('    const Eigen::Vector<double, ' + str(eta.shape[0]) + '> eta = this->GetInputPort("eta").Eval(context);\n')
    file.write('    const Eigen::Vector<double, ' + str(q.shape[0]) + '> q = context.get_continuous_state_vector().CopyToVector();\n')
    file.write('\n')
    for i, s in enumerate(eta):
        if s in list(dynamics.free_symbols):
            file.write(f'    const double {sp.ccode(s)} = eta[{i}];\n')
    file.write('\n')
    for i, s in enumerate(q):
        if s in list(dynamics.free_symbols):
            file.write(f'    const double {sp.ccode(s)} = q[{i}];\n')
    file.write('\n')
    for i, dyn in enumerate(dynamics):
        file.write(f'    derivatives->get_mutable_vector().SetAtIndex({i}, {sp.ccode(dyn)});\n')
    file.write(
'''}

void Model::eval(const drake::systems::Context<double> &context, drake::systems::BasicVector<double> *output) const {
'''
    )
    file.write('    const Eigen::Vector<double, ' + str(eta.shape[0]) + '> eta = this->GetInputPort("eta").Eval(context);\n')
    file.write('    const Eigen::Vector<double, ' + str(q.shape[0]) + '> q = context.get_continuous_state_vector().CopyToVector();\n')
    file.write('\n')
    for i, s in enumerate(eta):
        if s in list(dynamics.free_symbols):
            file.write(f'    const double {sp.ccode(s)} = eta[{i}];\n')
    file.write('\n')
    for i, s in enumerate(q):
        if s in list(dynamics.free_symbols):
            file.write(f'    const double {sp.ccode(s)} = q[{i}];\n')
    file.write(
'''
    Eigen::Vector<double, 14> state;

    state.segment(0, 7) = context.get_continuous_state_vector().CopyToVector();

    state.segment(7, 7) = Eigen::Vector<double, 7>{
'''
    )
    for i, dyn in enumerate(dynamics):
        file.write(f'        {sp.ccode(dyn)},\n')
    file.write(
'''    };

    output->SetFromVector(state);
}

const drake::systems::InputPort<double> & Model::get_control_input_port() const {
    return GetInputPort("eta");
}

const drake::systems::OutputPort<double> & Model::get_state_output_port() const {
    return GetOutputPort("q");
}
'''
    )
