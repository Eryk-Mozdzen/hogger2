#include <vector>

#include <QGridLayout>
#include <QGroupBox>
#include <QHBoxLayout>
#include <QPushButton>
#include <QTextEdit>

#include "Accelerometer.h"

static int getDirection(const Eigen::Vector3d vec) {
    const Eigen::Vector3d absolute = vec.cwiseAbs();

    if((absolute[0] > absolute[1]) && (absolute[0] > absolute[2])) {
        if(vec[0] > 0) {
            return 0;
        } else {
            return 3;
        }
    } else if((absolute[1] > absolute[0]) && (absolute[1] > absolute[2])) {
        if(vec[1] > 0) {
            return 1;
        } else {
            return 4;
        }
    } else {
        if(vec[2] > 0) {
            return 2;
        }
    }

    return 5;
}

Accelerometer::Accelerometer(QWidget *parent)
    : Interface{"accelerometer", parent},
      scale{Eigen::Matrix3d::Identity()},
      offset{Eigen::Vector3d::Zero()} {

    const std::vector<std::pair<QString, Eigen::Vector3d>> directions = {
        std::make_pair("+X", Eigen::Vector3d(g, 0, 0)),
        std::make_pair("+Y", Eigen::Vector3d(0, g, 0)),
        std::make_pair("+Z", Eigen::Vector3d(0, 0, g)),
        std::make_pair("-X", Eigen::Vector3d(-g, 0, 0)),
        std::make_pair("-Y", Eigen::Vector3d(0, -g, 0)),
        std::make_pair("-Z", Eigen::Vector3d(0, 0, -g)),
    };

    QGridLayout *grid = new QGridLayout(this);

    QPushButton *button_sample = new QPushButton("sample");
    QPushButton *button_undo = new QPushButton("undo");
    QTextEdit *line[6];

    grid->addWidget(button_sample, 1, 0);
    grid->addWidget(button_undo, 1, 1);

    for(int i = 0; i < 6; i++) {
        QGroupBox *group = new QGroupBox(directions[i].first);
        QGridLayout *layout = new QGridLayout(group);
        line[i] = new QTextEdit(this);

        line[i]->setReadOnly(true);
        line[i]->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding);

        layout->addWidget(line[i], 0, 0);
        grid->addWidget(group, 2 + (i % 3), i / 3);
    }

    connect(button_sample, &QPushButton::clicked, [this, directions, line]() {
        const int i = getDirection(current);

        samples.push_back(std::make_pair(directions[i].second, current));

        line[i]->append(
            QString::asprintf("% 5.4f % 5.4f % 5.4f", current(0), current(1), current(2)));

        leastSquares();
    });

    connect(button_undo, &QPushButton::clicked, [this, line]() {
        const auto last = samples.back();

        const int i = getDirection(last.second);

        line[i]->undo();

        samples.pop_back();

        leastSquares();
    });
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
