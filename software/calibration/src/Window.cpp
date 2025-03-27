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
#include "Publisher.h"
#include "Subscriber.h"
#include "Window.h"

Window::Window(QWidget *parent) : QWidget{parent}, current{nullptr} {
    QJsonObject message;
    message["config"] = QJsonArray();
    config = QJsonDocument(message);

    interfaces.push_back(new Magnetometer());
    interfaces.push_back(new Accelerometer());
    interfaces.push_back(new Gyroscope());

    QGridLayout *grid = new QGridLayout(this);

    Subscriber *subscriber = new Subscriber();
    Publisher *publisher = new Publisher();

    QThread *subscriberThread = new QThread();
    QThread *publisherThread = new QThread();

    subscriber->moveToThread(subscriberThread);
    publisher->moveToThread(publisherThread);

    connect(subscriberThread, &QThread::started, subscriber, &Subscriber::start);
    connect(subscriberThread, &QThread::finished, subscriber, &Subscriber::deleteLater);
    connect(subscriberThread, &QThread::finished, subscriberThread, &QThread::deleteLater);
    connect(subscriber, &Subscriber::messageReceived, this, &Window::receive);
    connect(this, &QObject::destroyed, subscriberThread, &QThread::quit);

    connect(publisherThread, &QThread::started, publisher, &Publisher::start);
    connect(publisherThread, &QThread::finished, publisher, &Publisher::deleteLater);
    connect(publisherThread, &QThread::finished, publisherThread, &QThread::deleteLater);
    connect(this, &Window::transmit, publisher, &Publisher::publish);
    connect(this, &QObject::destroyed, publisherThread, &QThread::quit);

    subscriberThread->start();
    publisherThread->start();

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

            QJsonObject json;
            json["config_req"] = QJsonValue::Null;
            transmit(QJsonDocument(json));
        });

        connect(button_update, &QPushButton::clicked, [&]() {
            if(current) {
                QJsonObject content = config.object()["config"].toObject();
                current->update(content);
                QJsonObject message;
                message["config"] = content;
                config.setObject(message);

                calibration_text->setText(config.toJson(QJsonDocument::Indented));
            } else {
                calibration_text->setText("app not selected");
            }
        });

        connect(button_set, &QPushButton::clicked, [&]() {
            calibration_text->setText("saving...");

            transmit(config);
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
        if(json.object()["telemetry"].isObject()) {
            const QJsonObject telemetry = json.object()["telemetry"].toObject();

            if(current) {
                current->receive(telemetry);
            }

            update();

            return;
        }
    }

    if(json.object().contains("config")) {
        calibration_text->setText(json.toJson(QJsonDocument::Indented));

        return;
    }
}
