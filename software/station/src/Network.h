#pragma once

#include <QWidget>
#include <QSettings>
#include <QJsonDocument>
#include <QComboBox>
#include <QPushButton>
#include <QGroupBox>
#include <QTcpSocket>
#include <QUdpSocket>

class Network : public QGroupBox {
    Q_OBJECT

	static constexpr int tcp_port = 3333;
	static constexpr int udp_port = 4444;

	QSettings settings;
	QTcpSocket *tcpSocket;
    QUdpSocket *udpSocket;
	QComboBox *listComboBox;
	QPushButton *scanButton;
	QPushButton *saveButton;

public slots:
	void transmit(const QJsonDocument &json);
	void deviceScan();

signals:
	void receive(const QJsonDocument &json);

public:
	Network(QWidget *parent = nullptr);
};
