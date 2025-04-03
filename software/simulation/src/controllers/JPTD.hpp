#pragma once

#include <drake/systems/framework/leaf_system.h>
#include <drake/systems/framework/diagram.h>

class JPTD : public drake::systems::Diagram<double> {
    class OutputFunction : public drake::systems::LeafSystem<double> {
        void eval(const drake::systems::Context<double> &context, drake::systems::BasicVector<double> *output) const;

    public:
        OutputFunction();
    };

    class FeedbackControl : public drake::systems::LeafSystem<double> {
        const double k1;
        const double k2;

        void eval(const drake::systems::Context<double> &context, drake::systems::BasicVector<double> *output) const;

    public:
        FeedbackControl(const double k1, const double k2);
    };

    class DynamicDecoupling : public drake::systems::LeafSystem<double> {
        void eval(const drake::systems::Context<double> &context, drake::systems::BasicVector<double> *output) const;

    public:
        DynamicDecoupling();
    };

public:
    JPTD(const double k1, const double k2);

    const drake::systems::InputPort<double> & get_state_input_port() const;
    const drake::systems::InputPort<double> & get_trajectory_input_port() const;
    const drake::systems::OutputPort<double> & get_control_output_port() const;
};
