#include <iostream>
#include <iomanip>

#include <QGridLayout>
#include <QThread>
#include <QTimer>
#include <QSlider>
#include <QPushButton>

#include "common/qt/Network.h"
#include "common/qt/InterfaceWidget.h"
#include "Window.h"

double lerp(const double in, const double in_min, const double in_max, const double out_min, const double out_max) {
    return (out_max - out_min)*(in - in_min)/(in_max - in_min) + out_min;
}

Window::Window(QWidget *parent) : QWidget(parent) {
    QGridLayout *layout = new QGridLayout(this);

    common::Network *network = new common::Network();

    connect(network, &common::Network::receive, this, &Window::receive);
    connect(this, &Window::transmit, network, &common::Network::transmit);

    common::InterfaceWidget *networkInterface = new common::InterfaceWidget("Network interface", this);

    connect(network, &common::Network::stats, networkInterface, &common::InterfaceWidget::stats);
    connect(network, &common::Network::status, networkInterface, &common::InterfaceWidget::status);
    connect(network, &common::Network::scanFinished, networkInterface, &common::InterfaceWidget::scanFinished);
    connect(networkInterface, &common::InterfaceWidget::scan, network, &common::Network::scanHosts);
    connect(networkInterface, &common::InterfaceWidget::change, network, &common::Network::changeHost);

    QThread *networkThread = new QThread(this);

    network->moveToThread(networkThread);
    connect(networkThread, &QThread::started, network, &common::Network::start);
    connect(networkThread, &QThread::finished, network, &common::Network::deleteLater);
    connect(networkThread, &QThread::finished, networkThread, &QThread::deleteLater);
    connect(this, &QObject::destroyed, networkThread, &QThread::quit);

    networkThread->start();

    networkInterface->forceScan();

    layout->addWidget(networkInterface, 0, 0);
    layout->addWidget(&gamepad, 1, 0);

    {
        sliders[0] = new QSlider(Qt::Orientation::Vertical);
        sliders[1] = new QSlider(Qt::Orientation::Vertical);
        sliders[2] = new QSlider(Qt::Orientation::Vertical);
        sliders[3] = new QSlider(Qt::Orientation::Vertical);

        for(QSlider *slider : sliders) {
            slider->setRange(-150, 150);
            slider->setTickInterval(1);
        }

        sliders[0]->setValue(settings.value("offset_lx").toInt());
        sliders[1]->setValue(settings.value("offset_ly").toInt());
        sliders[2]->setValue(settings.value("offset_rx").toInt());
        sliders[3]->setValue(settings.value("offset_ry").toInt());

        QPushButton *save = new QPushButton("Save");
        connect(save, &QPushButton::pressed, [this]() {
            settings.setValue("offset_lx", sliders[0]->value());
            settings.setValue("offset_ly", sliders[1]->value());
            settings.setValue("offset_rx", sliders[2]->value());
            settings.setValue("offset_ry", sliders[3]->value());
        });

        QGroupBox *group = new QGroupBox("Servo Offset");
        QGridLayout *box = new QGridLayout(group);

        box->addWidget(sliders[0], 0, 0);
        box->addWidget(sliders[1], 0, 1);
        box->addWidget(sliders[2], 0, 2);
        box->addWidget(sliders[3], 0, 3);
        box->addWidget(save, 1, 0, 1, 4);

        layout->addWidget(group, 0, 1, 2, 1);
    }

    {
        QTimer *timer = new QTimer();

        connect(timer, &QTimer::timeout, [this]() {
            const double v = -1.*gamepad.get(Gamepad::Analog::LY);
            const double w = -5.*gamepad.get(Gamepad::Analog::LX);

            constexpr double L = 0.13;  // m
            constexpr double R = 0.05;  // m
            constexpr double W = 100;   // rad/s

            const double v1 = v - L*w;
            const double v2 = v + L*w;

            const double a1 = std::asin(std::clamp(v1/(W*R), -1., 1.));
            const double a2 = std::asin(std::clamp(v2/(W*R), -1., 1.));

            uint32_t pwm[6];

            pwm[0] = sliders[0]->value() + lerp(+a1, -M_PI, M_PI, 1000, 2000);
            pwm[1] = sliders[1]->value() + 1500;
            pwm[2] = gamepad.get(Gamepad::Button::A) ? 1200 : 1000;

            pwm[3] = sliders[2]->value() + lerp(-a2, -M_PI, M_PI, 1000, 2000);
            pwm[4] = sliders[3]->value() + 1500;
            pwm[5] = gamepad.get(Gamepad::Button::A) ? 1200 : 1000;

            transmit(0x02, QByteArray(reinterpret_cast<char *>(&pwm), sizeof(pwm)));
        });

        timer->start(20);
    }
}

void Window::receive(const uint8_t id, const double time, const QByteArray &payload) {
    if(id==0x01) {
        const int minutes = static_cast<int>(time) / 60;
        const double seconds = time - 60*minutes;

        std::cout << "[ ";
        std::cout << std::noshowpos << std::setfill('0') << std::setw(2) << minutes;
        std::cout << ":";
        std::cout << std::noshowpos << std::setfill('0') << std::setw(6) << std::setprecision(3) << std::fixed << seconds;
        std::cout << " ] ";

        std::cout << std::string(reinterpret_cast<const char *>(payload.data()), payload.size());
        std::cout << std::endl;
    }
}
