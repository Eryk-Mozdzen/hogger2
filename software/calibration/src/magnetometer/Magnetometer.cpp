#include <iomanip>
#include <iostream>

#include <Eigen/Dense>
#include <QHBoxLayout>

#include "Magnetometer.h"

Magnetometer::Magnetometer(QWidget *parent)
    : Interface("magnetometer", parent), raw{samples, this}, calibrated{samples, this} {
    scale = Eigen::Matrix3d::Identity();
    offset = Eigen::Vector3d::Zero();

    QHBoxLayout *layout = new QHBoxLayout(this);
    layout->setContentsMargins(0, 0, 0, 0);
    layout->setSpacing(0);
    layout->addWidget(&raw);
    layout->addWidget(&calibrated);
}

Interface *Magnetometer::create() const {
    return new Magnetometer();
}

void Magnetometer::leastSquares() {
    Eigen::MatrixXd J(samples.size(), 10);

    for(size_t i = 0; i < samples.size(); i++) {
        J(i, 0) = samples[i](0) * samples[i](0);
        J(i, 1) = samples[i](1) * samples[i](1);
        J(i, 2) = samples[i](2) * samples[i](2);
        J(i, 3) = samples[i](0) * samples[i](1);
        J(i, 4) = samples[i](1) * samples[i](2);
        J(i, 5) = samples[i](2) * samples[i](0);
        J(i, 6) = samples[i](0);
        J(i, 7) = samples[i](1);
        J(i, 8) = samples[i](2);
        J(i, 9) = -1;
    }

    const Eigen::JacobiSVD svd(J, Eigen::ComputeFullV);
    const Eigen::VectorXd p = svd.matrixV().col(9);

    Eigen::Matrix3d M;
    M << p(0), p(3) / 2.0, p(5) / 2.0, p(3) / 2.0, p(1), p(4) / 2.0, p(5) / 2.0, p(4) / 2.0, p(2);

    const Eigen::Vector3d v(p(6), p(7), p(8));

    const Eigen::Vector3d b = -M.inverse() * v / 2.0;

    const double d = b.dot(M * b) - p(9);
    const Eigen::Matrix3d M_normalized = M / d;

    Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> eigensolver(M_normalized);

    const Eigen::Matrix3d R = eigensolver.eigenvectors();
    const Eigen::Vector3d D = eigensolver.eigenvalues();

    const Eigen::DiagonalMatrix<double, 3> S(D.cwiseSqrt());

    const Eigen::Matrix3d A = R * S * R.transpose();

    scale = A;
    offset = -A * b;
}

void Magnetometer::receive(const QJsonObject &sensor) {
    if(sensor.contains("magnetometer")) {
        if(sensor["magnetometer"].isObject()) {
            const QJsonObject mag = sensor["magnetometer"].toObject();

            if(mag.contains("raw")) {
                if(mag["raw"].isArray()) {
                    const QJsonArray raw = mag["raw"].toArray();

                    const Eigen::Vector3d s = {
                        raw[0].toDouble(),
                        raw[1].toDouble(),
                        raw[2].toDouble(),
                    };

                    samples.push_back(s);

                    if(samples.size() > 10) {
                        leastSquares();
                    }

                    calibrated.set(scale, offset);
                }
            }
        }
    }
}

void Magnetometer::update(QJsonObject &calibration) const {
    calibration["magnetometer_scale"] = QJsonArray({
        scale(0, 0),
        scale(0, 1),
        scale(0, 2),
        scale(1, 0),
        scale(1, 1),
        scale(1, 2),
        scale(2, 0),
        scale(2, 1),
        scale(2, 2),
    });

    calibration["magnetometer_offset"] = QJsonArray({
        offset(0),
        offset(1),
        offset(2),
    });
}
