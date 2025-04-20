#pragma once

#include <drake/systems/framework/leaf_system.h>
#include <drake/systems/framework/diagram.h>

class JPTD : public drake::systems::Diagram<double> {
    class Smoother : public drake::systems::LeafSystem<double> {
        static constexpr double pi = 3.14159265359;

        const double x0;
        const double y0;
        const double theta0;
        const double T;

        mutable double heading;
        mutable int k;

        void eval(const drake::systems::Context<double> &context, drake::systems::BasicVector<double> *output) const;

    public:
        Smoother(const double x0, const double y0, const double theta0, const double T);
    };

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
    JPTD(const double x0, const double y0, const double theta0, const double T, const double k1, const double k2);

    const drake::systems::InputPort<double> & get_state_input_port() const;
    const drake::systems::InputPort<double> & get_trajectory_input_port() const;
    const drake::systems::OutputPort<double> & get_reference_output_port() const;
    const drake::systems::OutputPort<double> & get_control_output_port() const;
};
