#pragma once

#include "TrajectoryGenerator.hpp"

class Circle : public TrajectoryGenerator {
	const double R;
	const double T;

	Eigen::VectorX<double> value(const double &t) const;

public:
	Circle(const double R, const double T);
};
