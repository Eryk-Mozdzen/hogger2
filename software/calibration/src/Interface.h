#pragma once

#include <QJsonArray>
#include <QJsonDocument>
#include <QJsonObject>
#include <QWidget>

class Interface : public QWidget {
    QString name;

public:
    Interface(QString name, QWidget *parent = nullptr) : QWidget{parent}, name{name} {
    }

    QString getName() const {
        return name;
    }

    virtual Interface *create() const = 0;
    virtual void receive(const QJsonDocument &sensor) = 0;
    virtual void update(QJsonDocument &calibration) const = 0;
};
