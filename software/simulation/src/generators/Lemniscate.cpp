#include "Lemniscate.hpp"
#include "control/generators.h"

Lemniscate::Lemniscate(const double a, const double T) : TrajectoryGenerator{3, 4}, a{a}, T{T} {

}

Eigen::VectorX<double> Lemniscate::value(const double &t) const {
    const float params[2] = {
        static_cast<float>(a),
        static_cast<float>(T),
    };

    float hd[12];
    generators_lemniscate(hd, params, static_cast<float>(t));

    Eigen::Vector<double, 12> trajectory;

    for(int i=0; i<12; i++) {
        trajectory[i] = hd[i];
    }

    return trajectory;
}
