#include <QJsonArray>
#include <QJsonDocument>
#include <QProcess>
#include <QTcpSocket>
#include <QTimer>

#include "Visualizer.h"

Visualizer::Visualizer(QObject *parent) : QObject{parent} {
    setlocale(LC_NUMERIC, "en_US.UTF-8");
}

void Visualizer::start() {
    socket = new QTcpSocket(this);
}

void Visualizer::spawn() {
    QProcess *process = new QProcess(this);

    connect(process, &QProcess::finished, process, [this, process]() {
        finished();
        process->deleteLater();
    });

    process->start("/home/emozdzen/repos/visualization-3d/build/server");

    QTimer::singleShot(1000, [this]() {
        socket->connectToHost("localhost", 8080);
        socket->waitForConnected();

        write(QJsonObject{
            {"command", "clear"}
        });
        write(QJsonObject{
            {"command", "config"},
            {"theme",   "light" },
            {"camera",  "orbit" }
        });
        write(QJsonObject{
            {"command",  "create"                                                      },
            {"path",     "obj1"                                                        },
            {"geometry",
             QJsonObject{{"shape", "cuboid"}, {"size", QJsonArray{{0.5, 0.25, 0.1}}}}},
            {"material", QJsonObject{{"color", QJsonArray{{255, 0, 0}}}}               }
        });
    });
}

void Visualizer::receive(const QJsonDocument &json) {
    if(!json.isObject()) {
        return;
    }

    if(!json.object().contains("telemetry")) {
        return;
    }

    const QJsonObject telemetry = json.object()["telemetry"].toObject();
    const QJsonObject estimate = telemetry["estimate"].toObject();

    write(QJsonObject{
        {"command",   "update"                                                            },
        {"path",      "obj1"                                                              },
        {"transform",
         QJsonObject{
             {"translation",
              QJsonArray{{estimate["pos"][0].toDouble(), estimate["pos"][1].toDouble(), 0}}},
             {"rpy", QJsonArray{{0., 0., estimate["theta"].toDouble() * 180.0 / 3.1415}}}}}
    });
}

void Visualizer::write(const QJsonObject &json) {
    if(socket->state() != QAbstractSocket::SocketState::ConnectedState) {
        return;
    }

    const QJsonDocument document(json);
    const QString text = document.toJson(QJsonDocument::JsonFormat::Compact);

    socket->write(text.toUtf8());
    socket->flush();
}
