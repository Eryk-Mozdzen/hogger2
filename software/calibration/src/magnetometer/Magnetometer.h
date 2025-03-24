#pragma once

#include <vector>

#include <Eigen/Dense>

#include "Interface.h"
#include "Viewer.h"
#include "utils.h"

class Magnetometer : public Interface {
    Viewer raw;
    Viewer calibrated;

    std::vector<Sample> samples;

    Eigen::Matrix3d scale;
    Eigen::Vector3d offset;

public:
    Magnetometer(QWidget *parent = nullptr);

    Interface *create() const;
    void receive(const QJsonObject &sensor);
    void update(QJsonObject &calibration) const;
};
