#include "Circle.hpp"

Circle::Circle(const double x, const double y, const double R, const double T) : TrajectoryGenerator{5, 3}, x{x}, y{y}, R{R}, w{2*pi/T} {

}

Eigen::VectorX<double> Circle::value(const double &t) const {
    Eigen::Vector<double, 5*3> trajectory;

    trajectory.segment(0, 5) = Eigen::Vector<double, 5>{
        x + R*cos(w*t),
        y + R*sin(w*t),
        w*t + pi/2 - pi/4, // very important -pi/4 !!!
        -300*t,
        +300*t,
    };

    trajectory.segment(5, 5) = Eigen::Vector<double, 5>{
        -R*w*sin(w*t),
        R*w*cos(w*t),
        w,
        -300,
        +300,
    };

    trajectory.segment(10, 5) = Eigen::Vector<double, 5>{
        -R*w*w*cos(w*t),
        -R*w*w*sin(w*t),
        0,
        0,
        0,
    };

    return trajectory;
}
