#include <vector>

#include <QGridLayout>
#include <QGroupBox>
#include <QHBoxLayout>
#include <QPushButton>
#include <QTextEdit>

#include "Accelerometer.h"

Accelerometer::Accelerometer(QWidget *parent)
    : Interface{"accelerometer", parent},
      scale{Eigen::Matrix3d::Identity()},
      offset{Eigen::Vector3d::Zero()} {
    const std::vector<std::pair<QString, Eigen::Vector3d>> predefined = {
        std::make_pair("+X", Eigen::Vector3d(g, 0, 0)),
        std::make_pair("+Y", Eigen::Vector3d(0, g, 0)),
        std::make_pair("+Z", Eigen::Vector3d(0, 0, g)),
        std::make_pair("-X", Eigen::Vector3d(-g, 0, 0)),
        std::make_pair("-Y", Eigen::Vector3d(0, -g, 0)),
        std::make_pair("-Z", Eigen::Vector3d(0, 0, -g))};

    QGridLayout *grid = new QGridLayout(this);

    for(int i = 0; i < 6; i++) {
        const Eigen::Vector3d orientation = predefined[i].second;

        QGroupBox *group = new QGroupBox(predefined[i].first);
        QGridLayout *layout = new QGridLayout(group);
        QTextEdit *line = new QTextEdit(this);
        QPushButton *sample = new QPushButton("sample");
        QPushButton *undo = new QPushButton("undo");

        line->setReadOnly(true);
        line->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding);

        connect(sample, &QPushButton::clicked, [this, orientation, line]() {
            samples.push_back(std::make_pair(orientation, current));

            line->append(
                QString::asprintf("% 5.4f % 5.4f % 5.4f", current(0), current(1), current(2)));

            leastSquares();
        });

        connect(undo, &QPushButton::clicked, [this, orientation, line]() {
            const auto it =
                std::find_if(samples.rbegin(), samples.rend(), [orientation](const auto &pair) {
                    return pair.second.isApprox(orientation);
                });

            if(it != samples.rend()) {
                samples.erase(it.base() - 1);
            }

            line->undo();

            leastSquares();
        });

        layout->addWidget(sample, 2, 0);
        layout->addWidget(undo, 3, 0);
        layout->addWidget(line, 0, 1, 6, 1);
        grid->addWidget(group, i % 3, i / 3);
    }
}

void Accelerometer::leastSquares() {
    const int N = samples.size();

    Eigen::MatrixXd X(N, 4);
    Eigen::MatrixXd b(N, 3);

    for(int i = 0; i < N; i++) {
        const Eigen::Vector3d &orientation = samples[i].first;
        const Eigen::Vector3d &reading = samples[i].second;

        X(i, 3) = 1.;
        X.block(i, 0, 1, 3) = reading.transpose();
        b.block(i, 0, 1, 3) = orientation.transpose();
    }

    const Eigen::MatrixXd coeff = (X.transpose() * X).inverse() * X.transpose() * b;

    scale = coeff.topRows(3).transpose();
    offset = coeff.row(3);
}

Interface *Accelerometer::create() const {
    return new Accelerometer();
}

void Accelerometer::receive(const QJsonObject &sensor) {
    if(sensor.contains("accelerometer")) {
        if(sensor["accelerometer"].isObject()) {
            const QJsonObject accel = sensor["accelerometer"].toObject();

            if(accel.contains("raw")) {
                if(accel["raw"].isArray()) {
                    const QJsonArray raw = accel["raw"].toArray();

                    current =
                        Eigen::Vector3d(raw[0].toDouble(), raw[1].toDouble(), raw[2].toDouble());
                }
            }
        }
    }
}

void Accelerometer::update(QJsonObject &calibration) const {
    calibration["accelerometer_scale"] = QJsonArray({
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

    calibration["accelerometer_offset"] = QJsonArray({
        offset(0),
        offset(1),
        offset(2),
    });
}
