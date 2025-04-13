#pragma once

#include "TrajectoryGenerator.hpp"

class Circle : public TrajectoryGenerator {
	static constexpr double pi = 3.14159265359f;

	const double x;
	const double y;
	const double R;
	const double w;

	Eigen::VectorX<double> value(const double &t) const;

public:
	Circle(const double x, const double y, const double R, const double T);
};
