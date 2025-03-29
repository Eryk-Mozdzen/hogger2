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
        J(i, 9) = 1;
    }

    const Eigen::JacobiSVD svd(J, Eigen::ComputeFullV);

    if(svd.info() != Eigen::Success) {
        return;
    }

    const Eigen::VectorXd p = svd.matrixV().col(9);
    const Eigen::VectorXd pp = p / p(9);

    const Eigen::Matrix3d A{
        {pp(0),     pp(3) / 2, pp(5) / 2},
        {pp(3) / 2, pp(1),     pp(4) / 2},
        {pp(5) / 2, pp(4) / 2, pp(2)    },
    };

    const Eigen::Vector3d b{
        pp(6) / 2,
        pp(7) / 2,
        pp(8) / 2,
    };

    const Eigen::Vector3d c = -A.inverse() * b;
    const Eigen::Matrix3d M = A / (c.transpose() * A * c);

    Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> solver(M);
    const Eigen::Matrix3d Q = solver.eigenvectors();
    const Eigen::Vector3d L = solver.eigenvalues();

    const Eigen::Matrix3d T = Q * L.cwiseSqrt().asDiagonal() * Q.transpose();

    scale = T;
    offset = -T * c;
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

                    leastSquares();

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
