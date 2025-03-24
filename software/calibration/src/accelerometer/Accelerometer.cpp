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
        QPushButton *clear = new QPushButton("clear");

        line->setReadOnly(true);
        line->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding);

        connect(sample, &QPushButton::clicked, [this, orientation, line]() {
            samples.push_back(std::make_pair(orientation, current));

            line->append(
                QString::asprintf("% 5.4f % 5.4f % 5.4f", current(0), current(1), current(2)));

            leastSquares();
        });

        connect(clear, &QPushButton::clicked, [this, orientation, line]() {
            samples.clear();

            line->setText("");

            leastSquares();
        });

        layout->addWidget(sample, 2, 0);
        layout->addWidget(clear, 3, 0);
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

void Accelerometer::receive(const QJsonDocument &sensor) {
    if(sensor.object().contains("accelerometer")) {
        const QJsonArray accel = sensor.object()["accelerometer"].toArray();

        current = Eigen::Vector3d(accel[0].toDouble(), accel[1].toDouble(), accel[2].toDouble());
    }
}

void Accelerometer::update(QJsonDocument &calibration) const {
    calibration.object()["accelerometer"] = QJsonArray({
        scale(0, 0),
        scale(0, 1),
        scale(0, 2),
        scale(1, 0),
        scale(1, 1),
        scale(1, 2),
        scale(2, 0),
        scale(2, 1),
        scale(2, 2),
        offset(0),
        offset(1),
        offset(2),
    });
}
