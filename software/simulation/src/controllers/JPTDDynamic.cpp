#include <drake/systems/framework/diagram_builder.h>
#include <drake/systems/primitives/demultiplexer.h>
#include <drake/systems/primitives/integrator.h>
#include <drake/systems/primitives/multiplexer.h>

#include "control/jptd_dynamic.h"
#include "control/robot_parameters.h"
#include "controllers/JPTDDynamic.hpp"

#define MOTOR_VEL 300

JPTDDynamic::OutputFunction::OutputFunction() {
    this->DeclareVectorInputPort("q", 9);
    this->DeclareVectorInputPort("eta3", 1);
    this->DeclareVectorInputPort("eta5", 1);
    this->DeclareVectorOutputPort("h", 10, &OutputFunction::eval);
}

void JPTDDynamic::OutputFunction::eval(const drake::systems::Context<double> &context,
                                       drake::systems::BasicVector<double> *output) const {
    const Eigen::VectorXd q = this->GetInputPort("q").Eval(context);

    const double eta_3 = this->GetInputPort("eta3").Eval(context)[0];
    const double eta_5 = this->GetInputPort("eta5").Eval(context)[0];

    const double theta = q[2];
    const double phi_1 = q[3];
    const double theta_1 = q[4];
    const double phi_2 = q[6];
    const double theta_2 = q[7];

    Eigen::Vector<double, 10> h;

    h.segment(0, 5) = Eigen::Vector<double, 5>{
        q[0], q[1], q[2], q[5], q[8],
    };

    h.segment(5, 5) = Eigen::Vector<double, 5>{
        ROBOT_PARAMETER_R * (sin(theta) * sin(theta_1) - cos(theta) * sin(phi_1) * cos(theta_1)) *
            eta_3,
        -ROBOT_PARAMETER_R * (cos(theta) * sin(theta_1) + sin(theta) * sin(phi_1) * cos(theta_1)) *
            eta_3,
        ROBOT_PARAMETER_R * (sin(phi_1) * cos(theta_1) / (2 * ROBOT_PARAMETER_L)) * eta_3 -
            ROBOT_PARAMETER_R * (sin(phi_2) * cos(theta_2) / (2 * ROBOT_PARAMETER_L)) * eta_5,
        eta_3,
        eta_5,
    };

    output->SetFromVector(h);
}

JPTDDynamic::FeedbackControl::FeedbackControl(const double k1, const double k2) : k1{k1}, k2{k2} {
    this->DeclareVectorInputPort("h", 10);
    this->DeclareVectorInputPort("hd", 12);
    this->DeclareVectorOutputPort("v", 5, &FeedbackControl::eval);
}

void JPTDDynamic::FeedbackControl::eval(const drake::systems::Context<double> &context,
                                        drake::systems::BasicVector<double> *output) const {
    const double t = context.get_time();
    const Eigen::VectorXd state = this->GetInputPort("h").Eval(context);
    const Eigen::VectorXd trajectory = this->GetInputPort("hd").Eval(context);

    const float h[] = {
        static_cast<float>(state[0]), static_cast<float>(state[1]), static_cast<float>(state[2]),
        static_cast<float>(state[3]), static_cast<float>(state[4]), static_cast<float>(state[5]),
        static_cast<float>(state[6]), static_cast<float>(state[7]), static_cast<float>(state[8]),
        static_cast<float>(state[9]),
    };

    const float hd[] = {
        static_cast<float>(trajectory[0]),
        static_cast<float>(trajectory[1]),
        static_cast<float>(trajectory[2]),
        static_cast<float>(-MOTOR_VEL * t),
        static_cast<float>(+MOTOR_VEL * t),
        static_cast<float>(trajectory[3]),
        static_cast<float>(trajectory[4]),
        static_cast<float>(trajectory[5]),
        -MOTOR_VEL,
        +MOTOR_VEL,
        static_cast<float>(trajectory[6]),
        static_cast<float>(trajectory[7]),
        static_cast<float>(trajectory[8]),
        0,
        0,
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
    jptd_dynamic_feedback_v(v, K1, K2, h, hd);

    const Eigen::Vector<double, 5> V{
        v[0], v[1], v[2], v[3], v[4],
    };

    output->SetFromVector(V);
}

JPTDDynamic::DynamicDecoupling::DynamicDecoupling() {
    this->DeclareVectorInputPort("v", 5);
    this->DeclareVectorInputPort("q", 9);
    this->DeclareVectorInputPort("eta3", 1);
    this->DeclareVectorInputPort("eta5", 1);
    this->DeclareVectorOutputPort("u", 5, &DynamicDecoupling::eval);
}

void JPTDDynamic::DynamicDecoupling::eval(const drake::systems::Context<double> &context,
                                          drake::systems::BasicVector<double> *output) const {
    const auto &V = this->GetInputPort("v").Eval(context);
    const auto &Q = this->GetInputPort("q").Eval(context);
    const auto &N3 = this->GetInputPort("eta3").Eval(context);
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
        0, 0, static_cast<float>(N3[0]), 0, static_cast<float>(N5[0]),
    };

    float u[5];
    jptd_dynamic_linearize_u(u, v, eta, q);

    const Eigen::Vector<double, 5> U{
        u[0], u[1], u[2], u[3], u[4],
    };

    output->SetFromVector(U);
}

JPTDDynamic::JPTDDynamic(const double k1, const double k2) {
    drake::systems::DiagramBuilder<double> builder;

    auto output = builder.AddSystem<OutputFunction>();
    auto feedback = builder.AddSystem<FeedbackControl>(k1, k2);
    auto decoupling = builder.AddSystem<DynamicDecoupling>();
    auto demux = builder.AddSystem<drake::systems::Demultiplexer>(5);
    auto integrator3 =
        builder.AddSystem<drake::systems::Integrator>(Eigen::Vector<double, 1>{-MOTOR_VEL});
    auto integrator5 =
        builder.AddSystem<drake::systems::Integrator>(Eigen::Vector<double, 1>{+MOTOR_VEL});
    auto mux = builder.AddSystem<drake::systems::Multiplexer>(5);

    builder.ExportInput(feedback->GetInputPort("hd"), "hd");
    builder.ExportInput(output->GetInputPort("q"), "q");
    builder.ExportOutput(mux->get_output_port(), "eta");

    builder.ConnectInput("q", decoupling->GetInputPort("q"));

    builder.Connect(output->GetOutputPort("h"), feedback->GetInputPort("h"));
    builder.Connect(feedback->GetOutputPort("v"), decoupling->GetInputPort("v"));
    builder.Connect(decoupling->GetOutputPort("u"), demux->get_input_port());

    builder.Connect(demux->get_output_port(0), mux->get_input_port(0));
    builder.Connect(demux->get_output_port(1), mux->get_input_port(1));
    builder.Connect(demux->get_output_port(2), integrator3->get_input_port());
    builder.Connect(demux->get_output_port(3), mux->get_input_port(3));
    builder.Connect(demux->get_output_port(4), integrator5->get_input_port());
    builder.Connect(integrator3->get_output_port(), mux->get_input_port(2));
    builder.Connect(integrator5->get_output_port(), mux->get_input_port(4));
    builder.Connect(integrator3->get_output_port(), decoupling->GetInputPort("eta3"));
    builder.Connect(integrator5->get_output_port(), decoupling->GetInputPort("eta5"));
    builder.Connect(integrator3->get_output_port(), output->GetInputPort("eta3"));
    builder.Connect(integrator5->get_output_port(), output->GetInputPort("eta5"));

    builder.BuildInto(this);
}

const drake::systems::InputPort<double> &JPTDDynamic::get_state_input_port() const {
    return GetInputPort("q");
}

const drake::systems::InputPort<double> &JPTDDynamic::get_trajectory_input_port() const {
    return GetInputPort("hd");
}

const drake::systems::OutputPort<double> &JPTDDynamic::get_control_output_port() const {
    return GetOutputPort("eta");
}
