#pragma once

#include <QWidget>
#include <QJsonDocument>

#include "LiveChart.h"

class Window : public QWidget {
    Q_OBJECT

    LiveChart *motor1;
    LiveChart *motor2;
    LiveChart *optical;

private slots:
    void receive(const QJsonDocument &json);

public:
    Window(QWidget *parent = nullptr);
};
