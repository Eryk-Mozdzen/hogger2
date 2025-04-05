#include <drake/systems/framework/diagram_builder.h>
#include <drake/systems/primitives/integrator.h>

#include "control/jptd_dynamic.h"
#include "controllers/JPTD.hpp"

JPTD::OutputFunction::OutputFunction() {
    this->DeclareVectorInputPort("q", 18);
    this->DeclareVectorOutputPort("h", 10, &OutputFunction::eval);
}

void JPTD::OutputFunction::eval(const drake::systems::Context<double> &context,
                                drake::systems::BasicVector<double> *output) const {
    const Eigen::VectorXd state = this->GetInputPort("q").Eval(context);

    Eigen::Vector<double, 10> h;

    h.segment(0, 5) = Eigen::Vector<double, 5>{
        state[0], state[1], state[2], state[5], state[8],
    };

    h.segment(5, 5) = Eigen::Vector<double, 5>{
        state[9], state[10], state[11], state[14], state[17],
    };

    output->SetFromVector(h);
}

JPTD::FeedbackControl::FeedbackControl(const double k1, const double k2) : k1{k1}, k2{k2} {
    this->DeclareVectorInputPort("h", 10);
    this->DeclareVectorInputPort("hd", 15);
    this->DeclareVectorOutputPort("v", 5, &FeedbackControl::eval);
}

void JPTD::FeedbackControl::eval(const drake::systems::Context<double> &context,
                                 drake::systems::BasicVector<double> *output) const {
    const Eigen::VectorXd state = this->GetInputPort("h").Eval(context);
    const Eigen::VectorXd trajectory = this->GetInputPort("hd").Eval(context);

    const float h[] = {
        static_cast<float>(state[0]), static_cast<float>(state[1]), static_cast<float>(state[2]),
        static_cast<float>(state[3]), static_cast<float>(state[4]),
    };

    const float d_h[] = {
        static_cast<float>(state[5]), static_cast<float>(state[6]), static_cast<float>(state[7]),
        static_cast<float>(state[8]), static_cast<float>(state[9]),
    };

    const float hd[] = {
        static_cast<float>(trajectory[0]), static_cast<float>(trajectory[1]),
        static_cast<float>(trajectory[2]), static_cast<float>(trajectory[3]),
        static_cast<float>(trajectory[4]),
    };

    const float d_hd[] = {
        static_cast<float>(trajectory[5]), static_cast<float>(trajectory[6]),
        static_cast<float>(trajectory[7]), static_cast<float>(trajectory[8]),
        static_cast<float>(trajectory[9]),
    };

    const float d2_hd[] = {
        static_cast<float>(trajectory[10]), static_cast<float>(trajectory[11]),
        static_cast<float>(trajectory[12]), static_cast<float>(trajectory[13]),
        static_cast<float>(trajectory[14]),
    };

    const float K1[] = {
        static_cast<float>(k1), 0, 0, 0, 0, 0, static_cast<float>(k1), 0, 0, 0, 0, 0,
        static_cast<float>(k1), 0, 0, 0, 0, 0, static_cast<float>(k1), 0, 0, 0, 0, 0,
        static_cast<float>(k1),
    };

    const float K2[] = {
        static_cast<float>(k2), 0, 0, 0, 0, 0, static_cast<float>(k2), 0, 0, 0, 0, 0,
        static_cast<float>(k2), 0, 0, 0, 0, 0, static_cast<float>(k2), 0, 0, 0, 0, 0,
        static_cast<float>(k2),
    };

    float v[5];
    jptd_dynamic_feedback_v(v, K1, K2, h, d_h, hd, d_hd, d2_hd);

    Eigen::Vector<double, 5> V{
        v[0], v[1], v[2], v[3], v[4],
    };

    output->SetFromVector(V);
}

JPTD::DynamicDecoupling::DynamicDecoupling() {
    this->DeclareVectorInputPort("v", 5);
    this->DeclareVectorInputPort("q", 18);
    this->DeclareVectorInputPort("eta", 5);
    this->DeclareVectorOutputPort("u", 5, &DynamicDecoupling::eval);
}

void JPTD::DynamicDecoupling::eval(const drake::systems::Context<double> &context,
                                   drake::systems::BasicVector<double> *output) const {
    const Eigen::VectorXd V = this->GetInputPort("v").Eval(context);
    const Eigen::VectorXd Q = this->GetInputPort("q").Eval(context);
    const Eigen::VectorXd N = this->GetInputPort("eta").Eval(context);

    const float v[] = {
        static_cast<float>(V[0]), static_cast<float>(V[1]), static_cast<float>(V[2]),
        static_cast<float>(V[3]), static_cast<float>(V[4]),
    };

    const float q[] = {
        static_cast<float>(Q[0]), static_cast<float>(Q[1]), static_cast<float>(Q[2]),
        static_cast<float>(Q[3]), static_cast<float>(Q[4]), static_cast<float>(Q[5]),
        static_cast<float>(Q[6]), static_cast<float>(Q[7]), static_cast<float>(Q[8]),
    };

    const float eta[] = {
        static_cast<float>(N[0]), static_cast<float>(N[1]), static_cast<float>(N[2]),
        static_cast<float>(N[3]), static_cast<float>(N[4]),
    };

    float u[5];
    jptd_dynamic_linearize_u(u, v, eta, q);

    Eigen::Vector<double, 5> U{
        u[0], u[1], u[2], u[3], u[4],
    };

    output->SetFromVector(U);
}

JPTD::JPTD(const double k1, const double k2) {
    drake::systems::DiagramBuilder<double> builder;

    auto output = builder.AddSystem<OutputFunction>();
    auto feedback = builder.AddSystem<FeedbackControl>(k1, k2);
    auto decoupling = builder.AddSystem<DynamicDecoupling>();
    auto integrator = builder.AddSystem<drake::systems::Integrator>(5);

    builder.ExportInput(feedback->GetInputPort("hd"), "hd");
    builder.ExportInput(output->GetInputPort("q"), "q");

    builder.ConnectInput("q", decoupling->GetInputPort("q"));

    builder.Connect(output->GetOutputPort("h"), feedback->GetInputPort("h"));
    builder.Connect(feedback->GetOutputPort("v"), decoupling->GetInputPort("v"));
    builder.Connect(integrator->get_output_port(), decoupling->GetInputPort("eta"));
    builder.Connect(decoupling->GetOutputPort("u"), integrator->get_input_port());

    builder.ExportOutput(integrator->get_output_port(), "eta");

    builder.BuildInto(this);
}

const drake::systems::InputPort<double> &JPTD::get_state_input_port() const {
    return GetInputPort("q");
}

const drake::systems::InputPort<double> &JPTD::get_trajectory_input_port() const {
    return GetInputPort("hd");
}

const drake::systems::OutputPort<double> &JPTD::get_control_output_port() const {
    return GetOutputPort("eta");
}
