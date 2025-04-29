import sympy as sp

x = sp.Symbol('x')
y = sp.Symbol('y')
theta = sp.Symbol('theta')
phi12d = sp.Symbol('phi_12d')
theta12d = sp.Symbol('theta_12d')
omegad = sp.Symbol('omega_d')

R = sp.Symbol('R')
L = sp.Symbol('L')

q = sp.Matrix([
    x,
    y,
    theta,
])

u = sp.Matrix([
    phi12d,
    theta12d,
    omegad,
])

dynamics = sp.Matrix([
    [R*omegad*(sp.cos(theta)*sp.cos(theta12d)*sp.sin(phi12d) + sp.sin(theta)*sp.sin(theta12d))],
    [R*omegad*(sp.sin(theta)*sp.cos(theta12d)*sp.sin(phi12d) - sp.cos(theta)*sp.sin(theta12d))],
    [0],
])

dynamics = sp.simplify(dynamics)

sp.pprint(dynamics)

dynamics = dynamics.subs([
    (R, sp.Symbol('ROBOT_PARAMETER_R')),
    (L, sp.Symbol('ROBOT_PARAMETER_L')),
])

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
    Model(const double x0, const double y0, const double theta0);

    const drake::systems::InputPort<double> & get_control_input_port() const;
    const drake::systems::OutputPort<double> & get_state_output_port() const;
};
'''
    )

with open(f'{here}/src/Model.cpp', 'w') as file:
    file.write(
'''#include "Model.hpp"
#include "control/robot_parameters.h"

Model::Model(const double x0, const double y0, const double theta0) {
'''
    )
    file.write('    this->DeclareContinuousState({x0, y0, theta0});\n')
    file.write('    this->DeclareVectorInputPort("u", 3);\n')
    file.write('    this->DeclareVectorOutputPort("q", 3, &Model::eval, {this->all_state_ticket()});\n')
    file.write(
'''}

void Model::DoCalcTimeDerivatives(const drake::systems::Context<double> &context, drake::systems::ContinuousState<double> *derivatives) const {
'''
    )
    file.write('    const Eigen::Vector<double, 3> u = this->GetInputPort("u").Eval(context);\n')
    file.write('    const Eigen::Vector<double, 3> q = context.get_continuous_state_vector().CopyToVector();\n')
    file.write('\n')
    for i, s in enumerate(u):
        if s in list(dynamics.free_symbols):
            file.write(f'    const double {sp.ccode(s)} = u[{i}];\n')
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
    output->SetFromVector(context.get_continuous_state_vector().CopyToVector());
}

const drake::systems::InputPort<double> & Model::get_control_input_port() const {
    return GetInputPort("u");
}

const drake::systems::OutputPort<double> & Model::get_state_output_port() const {
    return GetOutputPort("q");
}
'''
    )
