#pragma once

#include <QObject>
#include <QTcpSocket>
#include <QJsonObject>

class Visualizer : public QObject {
    Q_OBJECT

    double cameraPosition[3] = {0, 0, 0};
    float latlon_ref[2] = {0, 0};

    QTcpSocket *socket = nullptr;

    void write(const QJsonObject &json);

public slots:
    void receive(const QJsonDocument &json);
    void spawn();

signals:
    void finished();

public:
    Visualizer(QObject *parent = nullptr);
    void start();
};
