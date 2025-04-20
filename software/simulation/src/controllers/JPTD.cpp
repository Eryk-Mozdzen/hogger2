#include <drake/systems/framework/diagram_builder.h>
#include <drake/systems/primitives/demultiplexer.h>
#include <drake/systems/primitives/integrator.h>
#include <drake/systems/primitives/multiplexer.h>

#include "control/jptd_dynamic.h"
#include "control/robot_parameters.h"
#include "controllers/JPTD.hpp"

#define MOTOR_VEL 300

JPTD::Smoother::Smoother(const double x0, const double y0, const double theta0, const double T)
    : x0{x0}, y0{y0}, theta0{theta0}, T{T} {
    this->DeclareVectorInputPort("u", 3 * 4);
    this->DeclareVectorOutputPort("y", 3 * 4, &Smoother::eval);

    heading = theta0;
    k = 0;
}

void JPTD::Smoother::eval(const drake::systems::Context<double> &context,
                          drake::systems::BasicVector<double> *output) const {
    const double t = context.get_time();

    const Eigen::VectorXd U = this->GetInputPort("u").Eval(context);
    const float u[3 * 4] = {
        static_cast<float>(U[0]), static_cast<float>(U[1]),  static_cast<float>(U[2]),
        static_cast<float>(U[3]), static_cast<float>(U[4]),  static_cast<float>(U[5]),
        static_cast<float>(U[6]), static_cast<float>(U[7]),  static_cast<float>(U[8]),
        static_cast<float>(U[9]), static_cast<float>(U[10]), static_cast<float>(U[11]),
    };

    float y[3 * 4];
    if(t < T) {
        jptd_dynamic_smooth(y, x0, y0, theta0, u, T, t);
    } else {
        memcpy(y, u, sizeof(y));
    }

    const double same = std::abs(heading - (y[2] + 2 * (k + 0) * pi));
    const double more = std::abs(heading - (y[2] + 2 * (k + 1) * pi));
    const double less = std::abs(heading - (y[2] + 2 * (k - 1) * pi));

    if(more < same && more < less) {
        k++;
    } else if(less < same && less < more) {
        k--;
    }
    y[2] += 2 * k * pi;
    heading = y[2];

    const Eigen::Vector<double, 3 * 4> Y{
        y[0], y[1], y[2], y[3], y[4], y[5], y[6], y[7], y[8], y[9], y[10], y[11],
    };

    output->SetFromVector(Y);
}

JPTD::OutputFunction::OutputFunction() {
    this->DeclareVectorInputPort("q", 9);
    this->DeclareVectorInputPort("eta3", 1);
    this->DeclareVectorInputPort("eta5", 1);
    this->DeclareVectorOutputPort("h", 10, &OutputFunction::eval);
}

void JPTD::OutputFunction::eval(const drake::systems::Context<double> &context,
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
        R * (sin(theta) * sin(theta_1) - cos(theta) * sin(phi_1) * cos(theta_1)) * eta_3,
        -R * (cos(theta) * sin(theta_1) + sin(theta) * sin(phi_1) * cos(theta_1)) * eta_3,
        R * (sin(phi_1) * cos(theta_1) / (2 * L)) * eta_3 -
            R * (sin(phi_2) * cos(theta_2) / (2 * L)) * eta_5,
        eta_3,
        eta_5,
    };

    output->SetFromVector(h);
}

JPTD::FeedbackControl::FeedbackControl(const double k1, const double k2) : k1{k1}, k2{k2} {
    this->DeclareVectorInputPort("h", 10);
    this->DeclareVectorInputPort("hd", 12);
    this->DeclareVectorOutputPort("v", 5, &FeedbackControl::eval);
}

void JPTD::FeedbackControl::eval(const drake::systems::Context<double> &context,
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

JPTD::DynamicDecoupling::DynamicDecoupling() {
    this->DeclareVectorInputPort("v", 5);
    this->DeclareVectorInputPort("q", 9);
    this->DeclareVectorInputPort("eta3", 1);
    this->DeclareVectorInputPort("eta5", 1);
    this->DeclareVectorOutputPort("u", 5, &DynamicDecoupling::eval);
}

void JPTD::DynamicDecoupling::eval(const drake::systems::Context<double> &context,
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

JPTD::JPTD(const double x0,
           const double y0,
           const double theta0,
           const double T,
           const double k1,
           const double k2) {
    drake::systems::DiagramBuilder<double> builder;

    auto smoother = builder.AddSystem<Smoother>(x0, y0, theta0, T);
    auto output = builder.AddSystem<OutputFunction>();
    auto feedback = builder.AddSystem<FeedbackControl>(k1, k2);
    auto decoupling = builder.AddSystem<DynamicDecoupling>();
    auto demux = builder.AddSystem<drake::systems::Demultiplexer>(5);
    auto integrator3 =
        builder.AddSystem<drake::systems::Integrator>(Eigen::Vector<double, 1>{-MOTOR_VEL});
    auto integrator5 =
        builder.AddSystem<drake::systems::Integrator>(Eigen::Vector<double, 1>{+MOTOR_VEL});
    auto mux = builder.AddSystem<drake::systems::Multiplexer>(5);

    builder.ExportInput(smoother->get_input_port(), "hd");
    builder.ExportInput(output->GetInputPort("q"), "q");
    builder.ExportOutput(mux->get_output_port(), "eta");
    builder.ExportOutput(smoother->get_output_port(), "h_ref");

    builder.ConnectInput("q", decoupling->GetInputPort("q"));

    builder.Connect(smoother->get_output_port(), feedback->GetInputPort("hd"));
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

const drake::systems::InputPort<double> &JPTD::get_state_input_port() const {
    return GetInputPort("q");
}

const drake::systems::InputPort<double> &JPTD::get_trajectory_input_port() const {
    return GetInputPort("hd");
}

const drake::systems::OutputPort<double> &JPTD::get_reference_output_port() const {
    return GetOutputPort("h_ref");
}

const drake::systems::OutputPort<double> &JPTD::get_control_output_port() const {
    return GetOutputPort("eta");
}
