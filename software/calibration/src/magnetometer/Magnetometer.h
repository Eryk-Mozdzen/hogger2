#pragma once

#include <Eigen/Dense>
#include <vector>

#include "Interface.h"
#include "Viewer.h"

class Magnetometer : public Interface {
    Viewer raw;
    Viewer calibrated;

    std::vector<Eigen::Vector3d> samples;

    Eigen::Matrix3d scale;
    Eigen::Vector3d offset;

    void leastSquares();

public:
    Magnetometer(QWidget *parent = nullptr);

    Interface *create() const;
    void receive(const QJsonObject &sensor);
    void update(QJsonObject &calibration) const;
};
