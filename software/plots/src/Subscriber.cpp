#include <QByteArray>
#include <QDebug>
#include <QJsonDocument>
#include <QJsonParseError>
#include <QObject>
#include <QSocketNotifier>
#include <zmq.hpp>

#include "Subscriber.h"

Subscriber::Subscriber(const QString &endpoint, QObject *parent)
    : QObject(parent),
      m_endpoint(endpoint.toStdString()),
      m_context{1},
      m_socket{m_context, zmq::socket_type::sub} {
}

Subscriber::~Subscriber() {
    stop();
}

void Subscriber::start() {
    try {
        m_socket.connect(m_endpoint);
        m_socket.set(zmq::sockopt::subscribe, "");

        int fd = m_socket.get(zmq::sockopt::fd);
        m_notifier = new QSocketNotifier(fd, QSocketNotifier::Read, this);

        connect(m_notifier, &QSocketNotifier::activated, this, &Subscriber::receiveMessage);
    } catch(const zmq::error_t &e) {
        emit errorOccurred(QString("ZeroMQ error: %1").arg(e.what()));
    }
}

void Subscriber::stop() {
    if(m_notifier) {
        m_notifier->setEnabled(false);
        delete m_notifier;
        m_notifier = nullptr;
    }

    try {
        if(m_socket.handle() != nullptr) {
            m_socket.close();
        }
        if(m_context.handle() != nullptr) {
            m_context.close();
        }
    } catch(const zmq::error_t &e) {
        qWarning() << "Error closing socket:" << e.what();
    }
}

void Subscriber::receiveMessage() {
    while(true) {
        zmq::message_t msg;
        zmq::recv_result_t result;

        try {
            result = m_socket.recv(msg, zmq::recv_flags::dontwait);
            if(!result)
                break;
        } catch(const zmq::error_t &e) {
            if(e.num() != EAGAIN) {
                emit errorOccurred(QString("Receive error: %1").arg(e.what()));
            }
            break;
        }

        QByteArray data(static_cast<char *>(msg.data()), msg.size());
        QJsonParseError parseError;
        QJsonDocument doc = QJsonDocument::fromJson(data, &parseError);

        if(parseError.error != QJsonParseError::NoError) {
            emit errorOccurred("JSON parse error: " + parseError.errorString());
            continue;
        }

        emit messageReceived(doc);
    }
}
