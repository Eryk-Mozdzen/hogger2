#pragma once

#include <Eigen/Dense>
#include <QWidget>
#include <vector>

class Viewer : public QWidget {
    static constexpr int point = 5;
    static constexpr int size = 500;

    const std::vector<Eigen::Vector3d> &samples;
    Eigen::Matrix3d scale;
    Eigen::Vector3d offset;

    void paintEvent(QPaintEvent *event) override;

public:
    Viewer(const std::vector<Eigen::Vector3d> &s, QWidget *parent = nullptr);

    void set(const Eigen::Matrix3d &scale, const Eigen::Vector3d &offset);
};
