#include "Circle.hpp"
#include "control/generators.h"

Circle::Circle(const double R, const double T) : TrajectoryGenerator{3, 4}, R{R}, T{T} {

}

Eigen::VectorX<double> Circle::value(const double &t) const {
    const float params[2] = {
        static_cast<float>(R),
        static_cast<float>(T),
    };

    float hd[12];
    generators_circle(hd, params, static_cast<float>(t));

    Eigen::Vector<double, 12> trajectory;

    for(int i=0; i<12; i++) {
        trajectory[i] = hd[i];
    }

    return trajectory;
}
