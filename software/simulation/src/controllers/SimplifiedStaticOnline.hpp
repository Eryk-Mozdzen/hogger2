#pragma once

#include <drake/systems/framework/leaf_system.h>

class SimplifiedStaticOnline : public drake::systems::LeafSystem<double> {
    const double k;

    void eval(const drake::systems::Context<double> &context, drake::systems::BasicVector<double> *output) const;

public:
    SimplifiedStaticOnline(const double k);

    const drake::systems::InputPort<double> & get_state_input_port() const;
    const drake::systems::InputPort<double> & get_trajectory_input_port() const;
    const drake::systems::OutputPort<double> & get_control_output_port() const;
};
