#include <QNetworkInterface>
#include <QHostAddress>
#include <QProcess>
#include <QWidget>
#include <QTimer>
#include <QNetworkAccessManager>
#include <QNetworkRequest>
#include <QNetworkReply>
#include <QJsonObject>
#include <QJsonDocument>
#include <QVBoxLayout>
#include <QComboBox>
#include <QPushButton>

#include "Network.h"

Network::Network(QWidget *parent) : QGroupBox{"Network connection", parent}, started{false} {
    QVBoxLayout *layout = new QVBoxLayout(this);

    listComboBox = new QComboBox();
    listComboBox->setMinimumWidth(200);
    connect(listComboBox, &QComboBox::currentTextChanged, this, [this](QString device) {
        ip = device;
    });

    scanButton = new QPushButton("Scan IP");
    connect(scanButton, &QPushButton::pressed, this, &Network::deviceScan);

    saveButton = new QPushButton("Save as default");
    connect(saveButton, &QPushButton::pressed, [this]() {
        settings.setValue("defaultInput", listComboBox->currentText());
    });

    layout->addWidget(listComboBox);
    layout->addWidget(scanButton);
    layout->addWidget(saveButton);

    setLayout(layout);

    manager_get = new QNetworkAccessManager(this);
    manager_post = new QNetworkAccessManager(this);

    connect(manager_get, &QNetworkAccessManager::finished, this, [this](QNetworkReply *reply) {
        if(reply->error()==QNetworkReply::NoError) {
            const QByteArray bytes = reply->readAll().simplified().replace(" ", "").replace(0x00, "");
            const QJsonDocument document = QJsonDocument::fromJson(bytes);
            if(!document.isNull()) {
                receive(document);
            }
        }
        reply->deleteLater();
    });

    QTimer *timer = new QTimer(this);
    connect(timer, &QTimer::timeout, this, [this]() {
        if(started) {
            const QUrl url = QString("http://%1/get").arg(ip);

            manager_get->get(QNetworkRequest(url));
        }
    });
    timer->start(33);
}

void Network::transmit(const QJsonDocument &json) {
    if(started) {
        const QUrl url = QString("http://%1/post").arg(ip);
        const QByteArray data = json.toJson().simplified().replace(" ", "");

        QNetworkRequest request(url);
        request.setHeader(QNetworkRequest::ContentTypeHeader, "text/plain");
        manager_post->post(request, data);
    }
}

void Network::deviceScan() {
    listComboBox->setDisabled(true);
    scanButton->setDisabled(true);
    saveButton->setDisabled(true);
    listComboBox->clear();

    const QList<QNetworkInterface> allInterfaces = QNetworkInterface::allInterfaces();

    QList<QHostAddress> localAddresses;

    for(const QNetworkInterface &interface : allInterfaces) {
        if(interface.flags().testFlag(QNetworkInterface::IsUp) &&
            interface.flags().testFlag(QNetworkInterface::IsRunning) &&
            !interface.flags().testFlag(QNetworkInterface::IsLoopBack)) {
            for(const QNetworkAddressEntry &entry : interface.addressEntries()) {
                if(entry.ip().protocol()==QAbstractSocket::IPv4Protocol) {
                    localAddresses.append(entry.ip());
                }
            }
        }
    }

    QStringList *list = new QStringList();
    int *finished = new int(0);

    for(const QHostAddress &localAddress : localAddresses) {
        QProcess *nmapProcess = new QProcess(this);

        connect(nmapProcess, &QProcess::finished, [this, nmapProcess, list, finished, localAddresses](int exitCode, QProcess::ExitStatus exitStatus) {
            (void)exitCode;
            (void)exitStatus;

            const QString output = nmapProcess->readAllStandardOutput();

            const QRegularExpression regex(R"(Host:\s+(\d+\.\d+\.\d+\.\d+)\s+)");
            QRegularExpressionMatchIterator i = regex.globalMatch(output);

            while(i.hasNext()) {
                const QRegularExpressionMatch match = i.next();
                const QString address = match.captured(1);

                list->append(address);
            }

            (*finished)++;
            if(*finished==localAddresses.size()) {
                listComboBox->addItems(*list);
                const int index = listComboBox->findText(settings.value("defaultInput").toString());
                if(index != -1) {
                    listComboBox->setCurrentIndex(index);
                }
                listComboBox->setDisabled(false);
                scanButton->setDisabled(false);
                saveButton->setDisabled(false);

                started = true;

                delete finished;
                delete list;
            }

            nmapProcess->deleteLater();
        });

        QStringList nmapArgs;
        nmapArgs << "-sn";
        nmapArgs << "-n";
        nmapArgs << "-oG" << "-";
        nmapArgs << "--open";
        nmapArgs << "--noninteractive";
        nmapArgs << (localAddress.toString() + "/24");

        nmapProcess->start("nmap", nmapArgs);
    }
}
