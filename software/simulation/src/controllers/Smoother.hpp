#pragma once

#include <drake/systems/framework/leaf_system.h>

class Smoother : public drake::systems::LeafSystem<double> {
    static constexpr double pi = 3.14159265359;

    const double x0;
    const double y0;
    const double theta0;

    double *heading;
    int *k;

	void eval(const drake::systems::Context<double> &context, drake::systems::BasicVector<double> *output) const {
        const double t = context.get_time();
        const double T = 2;
        const double s0 = (t>=T) ? 1 : pow(t, 4)*(35*pow(T, 3) - 84*pow(T, 2)*t + 70*T*pow(t, 2) - 20*pow(t, 3))/pow(T, 7);
        const double s1 = (t>=T) ? 0 : 140*pow(t, 3)*(pow(T, 3) - 3*pow(T, 2)*t + 3*T*pow(t, 2) - pow(t, 3))/pow(T, 7);
        const double s2 = (t>=T) ? 0 : 420*pow(t, 2)*(pow(T, 3) - 4*pow(T, 2)*t + 5*T*pow(t, 2) - 2*pow(t, 3))/pow(T, 7);
        const double s3 = (t>=T) ? 0 : 840*t*(pow(T, 3) - 6*pow(T, 2)*t + 10*T*pow(t, 2) - 5*pow(t, 3))/pow(T, 7);

        const Eigen::VectorXd u = this->GetInputPort("u").Eval(context);
        const Eigen::VectorXd u0 = u.segment(0, 3);
        const Eigen::VectorXd u1 = u.segment(3, 3);
        const Eigen::VectorXd u2 = u.segment(6, 3);
        const Eigen::VectorXd u3 = u.segment(9, 3);

        const Eigen::Vector3d z0 {
            x0 + std::cos(theta0 - pi/4)*t,
            y0 + std::sin(theta0 - pi/4)*t,
            theta0,
        };

        const Eigen::Vector3d z1 {
            std::cos(theta0 - pi/4),
            std::sin(theta0 - pi/4),
            0,
        };

        const Eigen::Vector3d z2 {
            0,
            0,
            0,
        };

        const Eigen::Vector3d z3 {
            0,
            0,
            0,
        };

        Eigen::Vector3d y0 = z0 - s0*z0                                                         + s0*u0;
        Eigen::Vector3d y1 = z1 - s1*z0                         - s0*z1                         + s1*u0                         + s0*u1;
        Eigen::Vector3d y2 = z2 - s2*z0 - s1*z1                 - s1*z1 - s0*z2                 + s2*u0 + s1*u1                 + s1*u1 + s0*u2;
        Eigen::Vector3d y3 = z3 - s3*z0 - s2*z1 - s2*z1 - s1*z2 - s2*z1 - s1*z2 - s1*z2 - s0*z3 + s3*u0 + s2*u1 + s2*u1 + s1*u2 + s2*u1 + s1*u2 + s1*u2 + s0*u3;

        y0[2] = std::atan2(y1[1], y1[0]) + pi/4 + 2*(*k)*pi;
        y1[2] = (y1[0]*y2[1] - y1[1]*y2[0])/(pow(y1[0], 2) + pow(y1[1], 2));
        y2[2] = (-2*y1[0]*y2[1]*(y1[0]*y2[0] + y1[1]*y2[1]) + 2*y1[1]*y2[0]*(y1[0]*y2[0] + y1[1]*y2[1]) - (pow(y1[0], 2) + pow(y1[1], 2))*(-y1[0]*y3[1] + y1[1]*y3[0]))/pow(pow(y1[0], 2) + pow(y1[1], 2), 2);

        const double same = std::abs(*heading - y0[2]);
        const double more = std::abs(*heading - (y0[2] + 2*pi));
        const double less = std::abs(*heading - (y0[2] - 2*pi));

        if(more<same && more<less) {
            (*k)++;
            y0[2] +=2*pi;
        } else if(less<same && less<more) {
            (*k)--;
            y0[2] -=2*pi;
        }

        *heading = y0[2];

        Eigen::Vector<double, 3*4> y;
        y.segment(0, 3) = y0;
        y.segment(3, 3) = y1;
        y.segment(6, 3) = y2;
        y.segment(9, 3) = y3;

        output->SetFromVector(y);
	}

public:
    Smoother(const double x0, const double y0, const double theta0) : x0{x0}, y0{y0}, theta0{theta0} {
        this->DeclareVectorInputPort("u", 3*4);
        this->DeclareVectorOutputPort("y", 3*4, &Smoother::eval);

        heading = new double(theta0);
        k = new int(0);
	}

    ~Smoother() {
        delete heading;
        delete k;
    }
};
