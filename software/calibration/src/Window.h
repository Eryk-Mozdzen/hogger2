#pragma once

#include <vector>

#include <QHBoxLayout>
#include <QTextEdit>
#include <QWidget>

#include "Interface.h"

class Window : public QWidget {
    Q_OBJECT

    QJsonObject config;
    std::vector<Interface *> interfaces;
    Interface *current;
    QTextEdit *calibration_text;
    QHBoxLayout *interface_layout;

    void setCurrent(Interface *interface);

signals:
    void transmit(const QJsonDocument &json);

public slots:
    void receive(const QJsonDocument &json);

public:
    Window(QWidget *parent = nullptr);
};
