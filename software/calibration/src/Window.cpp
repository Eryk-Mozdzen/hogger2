#include <QGridLayout>
#include <QGroupBox>
#include <QHBoxLayout>
#include <QJsonDocument>
#include <QJsonObject>
#include <QPushButton>
#include <QTextEdit>
#include <QThread>
#include <QVBoxLayout>

#include "Accelerometer.h"
#include "Gyroscope.h"
#include "Magnetometer.h"
#include "Subscriber.h"
#include "Window.h"

Window::Window(QWidget *parent) : QWidget{parent}, current{nullptr} {
    interfaces.push_back(new Magnetometer());
    interfaces.push_back(new Accelerometer());
    interfaces.push_back(new Gyroscope());

    QGridLayout *grid = new QGridLayout(this);

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
        QGroupBox *group = new QGroupBox("applications");
        QHBoxLayout *layout = new QHBoxLayout(group);

        group->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Minimum);

        for(Interface *interface : interfaces) {
            QPushButton *button = new QPushButton(interface->getName(), group);

            button->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Fixed);

            connect(button, &QPushButton::clicked, std::bind(&Window::setCurrent, this, interface));

            layout->addWidget(button);
        }

        grid->addWidget(group, 0, 1);
    }

    {
        QGroupBox *group = new QGroupBox("interface");
        interface_layout = new QHBoxLayout(group);

        group->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding);

        grid->addWidget(group, 1, 1, 3, 1);
    }

    {
        QGroupBox *group = new QGroupBox("parameters");
        QVBoxLayout *layout = new QVBoxLayout(group);

        group->setFixedWidth(350);

        QFont font("System", 10);
        font.setStyleHint(QFont::TypeWriter);

        calibration_text = new QTextEdit(group);
        calibration_text->setReadOnly(true);
        calibration_text->setFont(font);
        QPushButton *button_read = new QPushButton("read from device", group);
        QPushButton *button_update = new QPushButton("update parameters", group);
        QPushButton *button_set = new QPushButton("write into device", group);

        connect(button_read, &QPushButton::clicked, [&]() {
            calibration_text->setText("fetching...");

            QJsonDocument json;
            json.object()["calibration"] = QJsonValue::Null;
            transmit(json);
        });

        connect(button_update, &QPushButton::clicked, [&]() {
            if(current) {
                current->update(calibration);

                calibration_text->setText(calibration.toJson(QJsonDocument::Indented));
            } else {
                calibration_text->setText("app not selected");
            }
        });

        connect(button_set, &QPushButton::clicked, [&]() {
            calibration_text->setText("saving...");

            QJsonDocument json;
            json.object()["calibration"] = calibration.object();
            transmit(json);
        });

        layout->addWidget(calibration_text);
        layout->addWidget(button_read);
        layout->addWidget(button_update);
        layout->addWidget(button_set);

        grid->addWidget(group, 0, 2, 4, 1);
    }
}

void Window::setCurrent(Interface *interface) {
    if(current) {
        interface_layout->removeWidget(current);
        delete current;
        current = nullptr;
    }

    if(interface) {
        current = interface->create();
    }

    if(current) {
        interface_layout->addWidget(current);
    }
}

void Window::receive(const QJsonDocument &json) {
    if(!json.isObject()) {
        return;
    }

    if(json.object().contains("telemetry")) {
        const QJsonDocument telemetry = QJsonDocument(json.object()["telemetry"].toObject());

        current->receive(telemetry);

        update();

        return;
    }

    if(json.object().contains("calibration")) {
        calibration = QJsonDocument(json.object()["calibration"].toObject());

        calibration_text->setText(calibration.toJson(QJsonDocument::Indented));

        return;
    }
}
