#include "controllers/SimplifiedStaticOnline.hpp"
#include "control/robot_parameters.h"
#include "control/simplified_static_online.h"

SimplifiedStaticOnline::SimplifiedStaticOnline(const double k) : k{k} {
    this->DeclareVectorInputPort("q", 9);
    this->DeclareVectorInputPort("hd", 12);
    this->DeclareVectorOutputPort("eta", 5, &SimplifiedStaticOnline::eval);
}

void SimplifiedStaticOnline::SimplifiedStaticOnline::eval(
    const drake::systems::Context<double> &context,
    drake::systems::BasicVector<double> *output) const {
    const auto &Q = this->GetInputPort("q").Eval(context);
    const auto &HD = this->GetInputPort("hd").Eval(context);

    const float K[] = {
        static_cast<float>(k), 0, 0, 0, 0, 0, static_cast<float>(k), 0, 0, 0, 0, 0,
        static_cast<float>(k), 0, 0, 0, 0, 0, static_cast<float>(k), 0, 0, 0, 0, 0,
        static_cast<float>(k),
    };

    const float q_full[] = {
        static_cast<float>(Q[0]), static_cast<float>(Q[1]), static_cast<float>(Q[2]),
        static_cast<float>(Q[3]), static_cast<float>(Q[4]), static_cast<float>(Q[5]),
        static_cast<float>(Q[6]), static_cast<float>(Q[7]), static_cast<float>(Q[8]),
    };

    const float hd[] = {
        static_cast<float>(HD[0]), static_cast<float>(HD[1]), 0.4, 0.02, 0.02,
        static_cast<float>(HD[3]), static_cast<float>(HD[4]), 0,   0,    0,
    };

    float eta_full[5];
    simplified_static_online_calculate(eta_full, K, q_full, hd);

    const Eigen::Vector<double, 5> eta{
        eta_full[0], eta_full[1], eta_full[2], eta_full[3], eta_full[4],
    };

    output->SetFromVector(eta);
}

const drake::systems::InputPort<double> &SimplifiedStaticOnline::get_state_input_port() const {
    return GetInputPort("q");
}

const drake::systems::InputPort<double> &SimplifiedStaticOnline::get_trajectory_input_port() const {
    return GetInputPort("hd");
}

const drake::systems::OutputPort<double> &SimplifiedStaticOnline::get_control_output_port() const {
    return GetOutputPort("eta");
}
