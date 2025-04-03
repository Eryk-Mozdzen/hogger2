#include <drake/systems/framework/diagram_builder.h>
#include <drake/systems/primitives/integrator.h>

#include "control/jptd_dynamic.h"
#include "controllers/JPTD.hpp"

JPTD::OutputFunction::OutputFunction() {
    this->DeclareVectorInputPort("q", 14);
    this->DeclareVectorOutputPort("h", 6, &OutputFunction::eval);
}

void JPTD::OutputFunction::eval(const drake::systems::Context<double> &context,
                                drake::systems::BasicVector<double> *output) const {
    const Eigen::VectorXd state = this->GetInputPort("q").Eval(context);

    Eigen::Vector<double, 6> h;

    h.segment(0, 3) = Eigen::Vector3d{
        state[0],
        state[1],
        state[2],
    };

    h.segment(3, 3) = Eigen::Vector3d{
        state[7],
        state[8],
        state[9],
    };

    output->SetFromVector(h);
}

JPTD::FeedbackControl::FeedbackControl(const double k1, const double k2) : k1{k1}, k2{k2} {
    this->DeclareVectorInputPort("h", 6);
    this->DeclareVectorInputPort("hd", 9);
    this->DeclareVectorOutputPort("v", 3, &FeedbackControl::eval);
}

void JPTD::FeedbackControl::eval(const drake::systems::Context<double> &context,
                                 drake::systems::BasicVector<double> *output) const {
    const Eigen::VectorXd state = this->GetInputPort("h").Eval(context);
    const Eigen::VectorXd trajectory = this->GetInputPort("hd").Eval(context);

    const float h[] = {
        static_cast<float>(state[0]),
        static_cast<float>(state[1]),
        static_cast<float>(state[2]),
    };

    const float d_h[] = {
        static_cast<float>(state[3]),
        static_cast<float>(state[4]),
        static_cast<float>(state[5]),
    };

    const float hd[] = {
        static_cast<float>(trajectory[0]),
        static_cast<float>(trajectory[1]),
        static_cast<float>(trajectory[2]),
    };

    const float d_hd[] = {
        static_cast<float>(trajectory[3]),
        static_cast<float>(trajectory[4]),
        static_cast<float>(trajectory[5]),
    };

    const float d2_hd[] = {
        static_cast<float>(trajectory[6]),
        static_cast<float>(trajectory[7]),
        static_cast<float>(trajectory[8]),
    };

    const float K1[] = {
        static_cast<float>(k1), 0, 0, 0, static_cast<float>(k1), 0, 0, 0, static_cast<float>(k1),
    };

    const float K2[] = {
        static_cast<float>(k2), 0, 0, 0, static_cast<float>(k2), 0, 0, 0, static_cast<float>(k2),
    };

    float v[3];
    jptd_dynamic_feedback_v(v, K1, K2, h, d_h, hd, d_hd, d2_hd);

    Eigen::Vector3d V{
        v[0],
        v[1],
        v[2],
    };

    output->SetFromVector(V);
}

JPTD::DynamicDecoupling::DynamicDecoupling() {
    this->DeclareVectorInputPort("v", 3);
    this->DeclareVectorInputPort("q", 14);
    this->DeclareVectorInputPort("eta", 3);
    this->DeclareVectorOutputPort("u", 3, &DynamicDecoupling::eval);
}

void JPTD::DynamicDecoupling::eval(const drake::systems::Context<double> &context,
                                   drake::systems::BasicVector<double> *output) const {
    const Eigen::VectorXd V = this->GetInputPort("v").Eval(context);
    const Eigen::VectorXd Q = this->GetInputPort("q").Eval(context);
    const Eigen::VectorXd N = this->GetInputPort("eta").Eval(context);

    const float v[] = {
        static_cast<float>(V[0]),
        static_cast<float>(V[1]),
        static_cast<float>(V[2]),
    };

    const float q[] = {
        static_cast<float>(Q[0]), static_cast<float>(Q[1]), static_cast<float>(Q[2]),
        static_cast<float>(Q[3]), static_cast<float>(Q[4]), static_cast<float>(Q[5]),
        static_cast<float>(Q[6]),
    };

    const float eta[] = {
        static_cast<float>(N[0]),
        static_cast<float>(N[1]),
        static_cast<float>(N[2]),
    };

    float u[3];
    jptd_dynamic_linearize_u(u, v, eta, q);

    Eigen::Vector3d U{
        u[0],
        u[1],
        u[2],
    };

    output->SetFromVector(U);
}

JPTD::JPTD(const double k1, const double k2) {
    drake::systems::DiagramBuilder<double> builder;

    auto output = builder.AddSystem<OutputFunction>();
    auto feedback = builder.AddSystem<FeedbackControl>(k1, k2);
    auto decoupling = builder.AddSystem<DynamicDecoupling>();
    auto integrator = builder.AddSystem<drake::systems::Integrator>(3);

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
