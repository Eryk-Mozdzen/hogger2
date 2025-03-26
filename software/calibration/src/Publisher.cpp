#include "Publisher.h"
#include <QByteArray>
#include <QDebug>

Publisher::Publisher(const QString &endpoint, QObject *parent)
    : QObject(parent),
      m_endpoint(endpoint.toStdString()),
      m_context{1},
      m_socket{m_context, zmq::socket_type::pub} {
}

Publisher::~Publisher() {
    stop();
}

void Publisher::start() {
    try {
        m_socket.connect(m_endpoint);
    } catch(const zmq::error_t &e) {
        emit errorOccurred(QString("ZeroMQ error: %1").arg(e.what()));
    }
}

void Publisher::stop() {
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

void Publisher::publish(const QJsonDocument &json) {
    try {
        const QByteArray data = json.toJson(QJsonDocument::Compact);
        zmq::message_t message(data.constData(), data.size());
        m_socket.send(message, zmq::send_flags::dontwait);
    } catch(const zmq::error_t &e) {
        emit errorOccurred(QString("Publish error: %1").arg(e.what()));
    }
}
