#include <QGridLayout>
#include <QTimer>
#include <QSlider>
#include <QPushButton>
#include <QJsonObject>
#include <QJsonArray>

#include "Network.h"
#include "Window.h"

double lerp(const double in, const double in_min, const double in_max, const double out_min, const double out_max) {
    return (out_max - out_min)*(in - in_min)/(in_max - in_min) + out_min;
}

Window::Window(QWidget *parent) : QWidget(parent) {
    QGridLayout *grid = new QGridLayout(this);

    Network *network = new Network();

    connect(network, &Network::receive, this, &Window::receive);
    connect(this, &Window::transmit, network, &Network::transmit);

    network->deviceScan();

    grid->addWidget(network, 0, 0);
    grid->addWidget(&joystick, 1, 0);

    {
        QGroupBox *group = new QGroupBox("State");
        QVBoxLayout *layout  = new QVBoxLayout(group);

        group->setMinimumWidth(350);
        group->setMinimumHeight(350);

        QFont font("System", 10);
        font.setStyleHint(QFont::TypeWriter);

        text[0] = new QTextEdit(this);
        text[0]->setReadOnly(true);
        text[0]->setFont(font);

        layout->addWidget(text[0]);

        grid->addWidget(group, 0, 2, 5, 1);
    }

    {
        QGroupBox *group = new QGroupBox("Controls");
        QVBoxLayout *layout  = new QVBoxLayout(group);

        group->setMinimumWidth(350);
        group->setMinimumHeight(350);

        QFont font("System", 10);
        font.setStyleHint(QFont::TypeWriter);

        text[1] = new QTextEdit(this);
        text[1]->setReadOnly(true);
        text[1]->setFont(font);

        layout->addWidget(text[1]);

        grid->addWidget(group, 0, 3, 5, 1);
    }

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

        grid->addWidget(group, 0, 1, 5, 1);
    }

    {
        QTimer *timer = new QTimer();

        connect(timer, &QTimer::timeout, [this]() {
            const double v = -1.*joystick.get(JoystickWidget::Analog::LY);
            const double w = -5.*joystick.get(JoystickWidget::Analog::LX);
            const double X = joystick.get(JoystickWidget::Analog::RX);

            constexpr double L = 0.13;  // m
            constexpr double R = 0.05;  // m
            constexpr double W = 200;   // rad/s

            const double v1 = v - L*w;
            const double v2 = v + L*w;

            const double a1 = std::asin(std::clamp(v1/(W*R), -1., 1.));
            const double a2 = std::asin(std::clamp(v2/(W*R), -1., 1.));

            const QJsonArray referenceConfiguration = {
                sliders[0]->value() + lerp(+a1, -M_PI, M_PI, 1000, 2000),
                sliders[1]->value() + 1500 + X*100,
                joystick.get(JoystickWidget::Button::A) ? 200 : 0,
                sliders[2]->value() + lerp(-a2, -M_PI, M_PI, 1000, 2000),
                sliders[3]->value() + 1500 - X*100,
                joystick.get(JoystickWidget::Button::A) ? 200 : 0,
            };

            QJsonObject json;
            json["command"] = "manual";
            json["reference_configuration"] = referenceConfiguration;

            const QJsonDocument document(json);

            text[1]->setText(document.toJson());

            transmit(document);
        });

        timer->start(20);
    }
}

void Window::receive(const QJsonDocument &json) {
    text[0]->setText(json.toJson());
}
