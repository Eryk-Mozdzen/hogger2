#include <QNetworkInterface>
#include <QHostAddress>
#include <QProcess>
#include <QWidget>
#include <QJsonObject>
#include <QJsonDocument>
#include <QVBoxLayout>
#include <QComboBox>
#include <QPushButton>
#include <QUdpSocket>

#include "Network.h"

Network::Network(QWidget *parent) : QGroupBox{"Network connection", parent} {
    QVBoxLayout *layout = new QVBoxLayout(this);

    listComboBox = new QComboBox();
    listComboBox->setMinimumWidth(200);
    connect(listComboBox, &QComboBox::currentTextChanged, this, [this](QString device) {
        ip = QHostAddress(device);
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

    txSocket = new QUdpSocket(this);
    rxSocket = new QUdpSocket(this);

    if(rxSocket->bind(rx_port, QUdpSocket::ShareAddress)) {
        connect(rxSocket, &QUdpSocket::readyRead, this, [this]() {
            while(rxSocket->hasPendingDatagrams()) {
                QByteArray datagram;
                datagram.resize(rxSocket->pendingDatagramSize());
                QHostAddress sender;
                quint16 senderPort;

                rxSocket->readDatagram(datagram.data(), datagram.size(), &sender, &senderPort);

                const QByteArray bytes = datagram.simplified().replace(" ", "").replace(0x00, "");
                const QJsonDocument document = QJsonDocument::fromJson(bytes);
                if(!document.isNull()) {
                    receive(document);
                }
            }
        });
    }
}

void Network::transmit(const QJsonDocument &json) {
    const QByteArray data = json.toJson(QJsonDocument::Compact);

    txSocket->writeDatagram(data, ip, tx_port);
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
