#pragma once

#include <QByteArray>
#include <QDebug>
#include <QJsonDocument>
#include <QJsonParseError>
#include <QObject>
#include <QSocketNotifier>
#include <zmq.hpp>

class Subscriber : public QObject {
    Q_OBJECT

    std::string m_endpoint;
    zmq::context_t m_context;
    zmq::socket_t m_socket;
    QSocketNotifier *m_notifier = nullptr;

public:
    explicit Subscriber(const QString &endpoint="tcp://localhost:6000", QObject *parent = nullptr);
    ~Subscriber() override;

public slots:
    void start();
    void stop();

signals:
    void messageReceived(const QJsonDocument &json);
    void errorOccurred(const QString &error);

private slots:
    void receiveMessage();
};
