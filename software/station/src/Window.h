#pragma once

#include <QWidget>
#include <QSlider>
#include <QSettings>
#include <QTextEdit>
#include <QJsonDocument>

#include "JoystickWidget.h"

class Window : public QWidget {
    Q_OBJECT

    QSlider *sliders[4];
    QTextEdit *text[2];
    QSettings settings;

    JoystickWidget joystick;

signals:
    void transmit(const QJsonDocument &json);

private slots:
    void receive(const QJsonDocument &json);

public:
    Window(QWidget *parent = nullptr);
};
