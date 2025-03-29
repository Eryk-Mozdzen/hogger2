#include <QPainter>
#include <cmath>

#include "Viewer.h"

Viewer::Viewer(const std::vector<Eigen::Vector3d> &s, QWidget *parent)
    : QWidget{parent}, samples{s} {
    scale = Eigen::Matrix3d::Identity();
    offset = Eigen::Vector3d::Zero();
    setFixedSize(size, size);
}

void Viewer::set(const Eigen::Matrix3d &scale, const Eigen::Vector3d &offset) {
    this->scale = scale;
    this->offset = offset;
}

void Viewer::paintEvent(QPaintEvent *event) {
    (void)event;

    double max = -1.f;
    for(const Eigen::Vector3d &sample : samples) {
        const Eigen::Vector3d conv = scale * sample + offset;

        max = std::max(max, std::abs((conv(0))));
        max = std::max(max, std::abs((conv(1))));
        max = std::max(max, std::abs((conv(2))));
    }
    const double window_scale = 0.4 * size / max;

    QPainter painter(this);
    painter.fillRect(rect(), Qt::transparent);
    painter.translate(rect().center());
    painter.setPen(QPen(Qt::transparent));

    for(const Eigen::Vector3d &sample : samples) {
        const Eigen::Vector3d conv = scale * sample + offset;

        const double x = window_scale * conv(0);
        const double y = window_scale * conv(1);
        const double z = window_scale * conv(2);

        painter.setBrush(Qt::red);
        painter.drawEllipse(x, y, point, point);
        painter.setBrush(Qt::darkGreen);
        painter.drawEllipse(y, z, point, point);
        painter.setBrush(Qt::blue);
        painter.drawEllipse(z, x, point, point);
    }
}
