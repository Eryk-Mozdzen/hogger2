#include <drake/systems/framework/diagram_builder.h>
#include <drake/systems/primitives/demultiplexer.h>
#include <drake/systems/primitives/discrete_derivative.h>
#include <drake/systems/primitives/integrator.h>
#include <drake/systems/primitives/multiplexer.h>

#include "control/jptd_dynamic.h"
#include "controllers/JPTD.hpp"

JPTD::OutputFunction::OutputFunction() {
    this->DeclareVectorInputPort("q", 9);
    this->DeclareVectorInputPort("dq", 9);
    this->DeclareVectorOutputPort("h", 10, &OutputFunction::eval);
}

void JPTD::OutputFunction::eval(const drake::systems::Context<double> &context,
                                drake::systems::BasicVector<double> *output) const {
    const Eigen::VectorXd q = this->GetInputPort("q").Eval(context);
    const Eigen::VectorXd dq = this->GetInputPort("dq").Eval(context);

    Eigen::Vector<double, 10> h;

    h.segment(0, 5) = Eigen::Vector<double, 5>{
        q[0], q[1], q[2], q[5], q[8],
    };

    h.segment(5, 5) = Eigen::Vector<double, 5>{
        dq[0], dq[1], dq[2], dq[5], dq[8],
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
    this->DeclareVectorInputPort("q", 9);
    this->DeclareVectorInputPort("eta1", 1);
    this->DeclareVectorInputPort("eta2", 1);
    this->DeclareVectorInputPort("eta3", 1);
    this->DeclareVectorInputPort("eta4", 1);
    this->DeclareVectorInputPort("eta5", 1);
    this->DeclareVectorOutputPort("u", 5, &DynamicDecoupling::eval);
}

void JPTD::DynamicDecoupling::eval(const drake::systems::Context<double> &context,
                                   drake::systems::BasicVector<double> *output) const {
    const auto &V = this->GetInputPort("v").Eval(context);
    const auto &Q = this->GetInputPort("q").Eval(context);
    const auto &N1 = this->GetInputPort("eta1").Eval(context);
    const auto &N2 = this->GetInputPort("eta2").Eval(context);
    const auto &N3 = this->GetInputPort("eta3").Eval(context);
    const auto &N4 = this->GetInputPort("eta4").Eval(context);
    const auto &N5 = this->GetInputPort("eta5").Eval(context);

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
        static_cast<float>(N1[0]), static_cast<float>(N2[0]), static_cast<float>(N3[0]),
        static_cast<float>(N4[0]), static_cast<float>(N5[0]),
    };

    //const float eta[] = {
    //    0, 0, static_cast<float>(N3[0]), 0, static_cast<float>(N5[0]),
    //};

    float u[5];
    jptd_dynamic_linearize_u(u, v, eta, q);

    Eigen::Vector<double, 5> U{
        u[0], u[1], u[2], u[3], u[4],
    };

    output->SetFromVector(U);
}

JPTD::JPTD(const double k1, const double k2) {
    drake::systems::DiagramBuilder<double> builder;

    auto derivative = builder.AddSystem<drake::systems::DiscreteDerivative>(9, 0.0001);
    auto output = builder.AddSystem<OutputFunction>();
    auto feedback = builder.AddSystem<FeedbackControl>(k1, k2);
    auto decoupling = builder.AddSystem<DynamicDecoupling>();
    auto demux = builder.AddSystem<drake::systems::Demultiplexer>(5);
    auto integrator1 = builder.AddSystem<drake::systems::Integrator>(1);
    auto integrator2 = builder.AddSystem<drake::systems::Integrator>(1);
    auto integrator3 = builder.AddSystem<drake::systems::Integrator>(Eigen::Vector<double, 1>{-300});
    auto integrator4 = builder.AddSystem<drake::systems::Integrator>(1);
    auto integrator5 = builder.AddSystem<drake::systems::Integrator>(Eigen::Vector<double, 1>{+300});
    auto mux = builder.AddSystem<drake::systems::Multiplexer>(5);

    builder.ExportInput(feedback->GetInputPort("hd"), "hd");
    builder.ExportInput(output->GetInputPort("q"), "q");
    builder.ExportOutput(mux->get_output_port(), "eta");

    builder.ConnectInput("q", decoupling->GetInputPort("q"));
    builder.ConnectInput("q", derivative->get_input_port());

    builder.Connect(derivative->get_output_port(), output->GetInputPort("dq"));
    builder.Connect(output->GetOutputPort("h"), feedback->GetInputPort("h"));
    builder.Connect(feedback->GetOutputPort("v"), decoupling->GetInputPort("v"));
    builder.Connect(decoupling->GetOutputPort("u"), demux->get_input_port());

    //builder.Connect(demux->get_output_port(0), mux->get_input_port(0));
    //builder.Connect(demux->get_output_port(1), mux->get_input_port(1));
    //builder.Connect(demux->get_output_port(2), integrator3->get_input_port());
    //builder.Connect(demux->get_output_port(3), mux->get_input_port(3));
    //builder.Connect(demux->get_output_port(4), integrator5->get_input_port());
    //builder.Connect(integrator3->get_output_port(), mux->get_input_port(2));
    //builder.Connect(integrator5->get_output_port(), mux->get_input_port(4));
    //builder.Connect(integrator3->get_output_port(), decoupling->GetInputPort("eta3"));
    //builder.Connect(integrator5->get_output_port(), decoupling->GetInputPort("eta5"));

    builder.Connect(demux->get_output_port(0), integrator1->get_input_port());
    builder.Connect(demux->get_output_port(1), integrator2->get_input_port());
    builder.Connect(demux->get_output_port(2), integrator3->get_input_port());
    builder.Connect(demux->get_output_port(3), integrator4->get_input_port());
    builder.Connect(demux->get_output_port(4), integrator5->get_input_port());
    builder.Connect(integrator1->get_output_port(), mux->get_input_port(0));
    builder.Connect(integrator2->get_output_port(), mux->get_input_port(1));
    builder.Connect(integrator3->get_output_port(), mux->get_input_port(2));
    builder.Connect(integrator4->get_output_port(), mux->get_input_port(3));
    builder.Connect(integrator5->get_output_port(), mux->get_input_port(4));
    builder.Connect(integrator1->get_output_port(), decoupling->GetInputPort("eta1"));
    builder.Connect(integrator2->get_output_port(), decoupling->GetInputPort("eta2"));
    builder.Connect(integrator3->get_output_port(), decoupling->GetInputPort("eta3"));
    builder.Connect(integrator4->get_output_port(), decoupling->GetInputPort("eta4"));
    builder.Connect(integrator5->get_output_port(), decoupling->GetInputPort("eta5"));

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
