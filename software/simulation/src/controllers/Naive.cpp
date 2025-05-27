#include "control/naive.h"
#include "control/robot_parameters.h"
#include "controllers/Naive.hpp"

#define MOTOR_VEL 300

Naive::Naive(const double k) : k{k} {
    this->DeclareVectorInputPort("hd", 12);
    this->DeclareVectorInputPort("q", 3);
    this->DeclareVectorOutputPort("u", 3, &Naive::eval);
}

void Naive::eval(const drake::systems::Context<double> &context, drake::systems::BasicVector<double> *output) const {
    const Eigen::VectorXd state = this->GetInputPort("q").Eval(context);
    const Eigen::VectorXd trajectory = this->GetInputPort("hd").Eval(context);

    const float q[] = {
        static_cast<float>(state[0]),
        static_cast<float>(state[1]),
        static_cast<float>(state[2]),
    };

    const float ref[] = {
        static_cast<float>(trajectory[0]),
        static_cast<float>(trajectory[1]),
        static_cast<float>(trajectory[3]),
        static_cast<float>(trajectory[4]),
    };

    float gimbal[2];
    naive_feedback(gimbal, MOTOR_VEL, q, ref, k);

    const Eigen::Vector<double, 3> U{
        gimbal[0], gimbal[1], MOTOR_VEL,
    };

    output->SetFromVector(U);
}

const drake::systems::InputPort<double> &Naive::get_state_input_port() const {
    return GetInputPort("q");
}

const drake::systems::InputPort<double> &Naive::get_trajectory_input_port() const {
    return GetInputPort("hd");
}

const drake::systems::OutputPort<double> &Naive::get_control_output_port() const {
    return GetOutputPort("u");
}
