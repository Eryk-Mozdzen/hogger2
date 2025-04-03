#include "Lemniscate.hpp"

Lemniscate::Lemniscate(const double c, const double T) : TrajectoryGenerator{3, 3}, a{c*sqrt2}, w{2*pi/T} {

}

Eigen::VectorX<double> Lemniscate::value(const double &t) const {
    Eigen::Vector<double, 3*3> trajectory;

    trajectory.segment(0, 3) = Eigen::Vector3d{
        a*cos(t*w)/(pow(sin(t*w), 2) + 1),
        a*sin(t*w)*cos(t*w)/(pow(sin(t*w), 2) + 1),
        fix(atan2(-a*w*pow(sin(t*w), 2)/(pow(sin(t*w), 2) + 1) + a*w*pow(cos(t*w), 2)/(pow(sin(t*w), 2) + 1) - 2*a*w*pow(sin(t*w), 2)*pow(cos(t*w), 2)/pow(pow(sin(t*w), 2) + 1, 2), -a*w*sin(t*w)/(pow(sin(t*w), 2) + 1) - 2*a*w*sin(t*w)*pow(cos(t*w), 2)/pow(pow(sin(t*w), 2) + 1, 2)))
    };

    trajectory.segment(3, 3) = Eigen::Vector3d{
        a*w*(pow(sin(t*w), 2) - 3)*sin(t*w)/pow(pow(sin(t*w), 2) + 1, 2),
        a*w*(1 - 3*pow(sin(t*w), 2))/pow(pow(sin(t*w), 2) + 1, 2),
        3*w*cos(t*w)/(pow(sin(t*w), 2) + 1)
    };

    trajectory.segment(6, 3) = Eigen::Vector3d{
        a*pow(w, 2)*(-pow(sin(t*w), 4) + 12*pow(sin(t*w), 2) - 3)*cos(t*w)/pow(pow(sin(t*w), 2) + 1, 3),
        2*a*pow(w, 2)*(14*sin(2*t*w) + 3*sin(4*t*w))/pow(cos(2*t*w) - 3, 3),
        pow(w, 2)*(-19*pow(sin(t*w), 10) + 31*pow(sin(t*w), 8) + 17*pow(sin(t*w), 6) - 33*pow(sin(t*w), 4)*pow(cos(t*w), 6) - 41*pow(sin(t*w), 4) - 10*pow(sin(t*w), 2) + 14*pow(cos(t*w), 10) - 2*pow(cos(t*w), 8) - 19*pow(cos(t*w), 6) - 2)*sin(t*w)/pow(pow(sin(t*w), 2) + 1, 4)
    };

    return trajectory;
}

double Lemniscate::fix(double angle) {
    while(angle>(270*deg2rad)) {
        angle -=2*pi;
    }

    while(angle<(-90*deg2rad)) {
        angle +=2*pi;
    }

    return angle;
}
