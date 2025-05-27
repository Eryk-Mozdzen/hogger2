#pragma once

#include "TrajectoryGenerator.hpp"

class Lemniscate : public TrajectoryGenerator {
	const double a;
	const double T;

	Eigen::VectorX<double> value(const double &t) const;

public:
	Lemniscate(const double a, const double T);
};
