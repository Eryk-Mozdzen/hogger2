#pragma once

#include <QJsonDocument>
#include <QWidget>

#include "LiveChart.h"

class Window : public QWidget {
    Q_OBJECT

    LiveChart *motor1;
    LiveChart *motor2;
    LiveChart *optical;
    LiveChart *accel;
    LiveChart *trajectory;

private slots:
    void receive(const QJsonDocument &json);

public:
    Window(QWidget *parent = nullptr);
};
