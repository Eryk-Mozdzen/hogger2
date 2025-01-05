#pragma once

#include <QWidget>
#include <QNetworkAccessManager>
#include <QSettings>
#include <QJsonDocument>
#include <QComboBox>
#include <QPushButton>

class Network : public QWidget {
    Q_OBJECT

	bool started;
	QString ip;
	QSettings settings;
    QNetworkAccessManager *manager;
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
