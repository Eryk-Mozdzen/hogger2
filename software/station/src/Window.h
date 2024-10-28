#pragma once

#include <QWidget>
#include <QSlider>
#include <QSettings>

#include "Gamepad.h"

class Window : public QWidget {
    Q_OBJECT

    QSlider *sliders[4];
    QSettings settings;

    Gamepad gamepad;

signals:
    void transmit(const uint8_t id, const QByteArray &payload);

private slots:
    void receive(const uint8_t id, const double time, const QByteArray &payload);

public:
    Window(QWidget *parent = nullptr);
};
