#include <cmath>
#include <filesystem>
#include <iostream>

#include <QCheckBox>
#include <QComboBox>
#include <QFormLayout>
#include <QGridLayout>
#include <QGroupBox>
#include <QJsonObject>
#include <QLabel>
#include <QLineEdit>
#include <QProcess>
#include <QPushButton>
#include <QRadioButton>
#include <QSettings>
#include <QSlider>
#include <QThread>
#include <QTimer>
#include <QVBoxLayout>

#include "LiveChart.h"
#include "Subscriber.h"
#include "Window.h"

Window::Window(QWidget *parent) : QWidget(parent) {
    QGridLayout *layout = new QGridLayout(this);

    Subscriber *subscriber = new Subscriber();
    QThread *subscriberThread = new QThread(this);

    subscriber->moveToThread(subscriberThread);

    connect(subscriberThread, &QThread::started, subscriber, &Subscriber::start);
    connect(subscriberThread, &QThread::finished, subscriber, &Subscriber::deleteLater);
    connect(subscriberThread, &QThread::finished, subscriberThread, &QThread::deleteLater);
    connect(subscriber, &Subscriber::messageReceived, this, &Window::receive);
    connect(this, &QObject::destroyed, subscriberThread, &QThread::quit);

    subscriberThread->start();

    {
        LiveChart::Config config;
        config.title = "motor 1";
        config.yLabel = "[rad/s]";
        config.yMin = 0;
        config.yMax = 600;
        config.yPrecision = 0;
        config.yTick = 50;

        motor1 = new LiveChart(config, this);
        motor1->addSeries("vel", QPen(Qt::black, 2, Qt::SolidLine));
        motor1->addSeries("vel_ref", QPen(Qt::black, 1, Qt::DashLine));
        motor1->addSeries("load", QPen(Qt::red, 2, Qt::SolidLine));

        layout->addWidget(motor1, 0, 0);
    }

    {
        LiveChart::Config config;
        config.title = "motor 2";
        config.yLabel = "[rad/s]";
        config.yMin = 0;
        config.yMax = 600;
        config.yPrecision = 0;
        config.yTick = 50;

        motor2 = new LiveChart(config, this);
        motor2->addSeries("vel", QPen(Qt::black, 2, Qt::SolidLine));
        motor2->addSeries("vel_ref", QPen(Qt::black, 1, Qt::DashLine));
        motor2->addSeries("load", QPen(Qt::red, 2, Qt::SolidLine));

        layout->addWidget(motor2, 1, 0);
    }

    {
        LiveChart::Config config;
        config.title = "optical flow";
        config.yLabel = "[m/s]";
        config.yMin = -1;
        config.yMax = 1;
        config.yPrecision = 2;
        config.yTick = 0.25;

        optical = new LiveChart(config, this);
        optical->addSeries("x", QPen(Qt::red, 1, Qt::SolidLine));
        optical->addSeries("y", QPen(Qt::green, 1, Qt::SolidLine));

        layout->addWidget(optical, 0, 1);
    }
}

void Window::receive(const QJsonDocument &json) {
    if(!json.isObject()) {
        return;
    }

    if(!json.object().contains("telemetry")) {
        return;
    }

    const QJsonObject telemetry = json.object()["telemetry"].toObject();
    const double time = telemetry["timestamp"].toDouble() * 0.001;

    const QJsonObject motor1_ = telemetry["motor_1"].toObject();
    motor1->append("vel", time, motor1_["vel"].toDouble());
    motor1->append("vel_ref", time, motor1_["vel_ref"].toDouble());
    motor1->append("load", time, motor1_["load"].toDouble() * 600);

    const QJsonObject motor2_ = telemetry["motor_2"].toObject();
    motor2->append("vel", time, motor2_["vel"].toDouble());
    motor2->append("vel_ref", time, motor2_["vel_ref"].toDouble());
    motor2->append("load", time, motor2_["load"].toDouble() * 600);

    constexpr double fh = 0.065;
    const QJsonArray optical_ = telemetry["optical_flow"].toArray();
    if(!optical_[0].isNull() && !optical_[1].isNull()) {
        optical->append("x", time, optical_[0].toDouble()*fh);
        optical->append("y", time, optical_[1].toDouble()*fh);
    }

    LiveChart::synchronize(time);
}
