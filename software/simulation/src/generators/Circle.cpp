#include "Circle.hpp"

Circle::Circle(const double x, const double y, const double R, const double T) : TrajectoryGenerator{3, 3}, x{x}, y{y}, R{R}, w{2*pi/T} {

}

Eigen::VectorX<double> Circle::value(const double &t) const {
    Eigen::Vector<double, 3*3> trajectory;

    trajectory.segment(0, 3) = Eigen::Vector3d{
        x + R*cos(w*t),
        y + R*sin(w*t),
        w*t + pi/2
    };

    trajectory.segment(3, 3) = Eigen::Vector3d{
        -R*w*sin(w*t),
        R*w*cos(w*t),
        w
    };

    trajectory.segment(6, 3) = Eigen::Vector3d{
        -R*w*w*cos(w*t),
        -R*w*w*sin(w*t),
        0
    };

    return trajectory;
}
