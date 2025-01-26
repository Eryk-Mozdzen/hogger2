#pragma once

#include <QWidget>
#include <QSettings>
#include <QJsonDocument>
#include <QComboBox>
#include <QPushButton>
#include <QGroupBox>
#include <QUdpSocket>

class Network : public QGroupBox {
    Q_OBJECT

	static constexpr int tx_port = 3333;
	static constexpr int rx_port = 4444;

	QSettings settings;
	QHostAddress ip;
    QUdpSocket *rxSocket;
	QUdpSocket *txSocket;
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
