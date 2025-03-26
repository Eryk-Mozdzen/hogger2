#pragma once

#include <QJsonDocument>
#include <QObject>
#include <zmq.hpp>

class Publisher : public QObject {
    Q_OBJECT

    std::string m_endpoint;
    zmq::context_t m_context;
    zmq::socket_t m_socket;

public:
    explicit Publisher(const QString &endpoint = "tcp://localhost:7000", QObject *parent = nullptr);
    ~Publisher() override;

public slots:
    void start();
    void stop();
    void publish(const QJsonDocument &json);

signals:
    void errorOccurred(const QString &error);
};
