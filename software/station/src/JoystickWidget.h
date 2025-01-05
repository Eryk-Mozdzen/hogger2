#pragma once

#include <QWidget>
#include <QGroupBox>
#include <QComboBox>
#include <QPushButton>
#include <QSettings>

#include "Joystick.h"

class JoystickWidget : public QGroupBox {
    Q_OBJECT

    QComboBox *addressComboBox;
    QSettings settings;

    utils::Joystick *joystick = nullptr;

private slots:
    void scanInput();
	void changeInput(int index);

public:
    enum Analog {
        LX = 0,
        LY = 1,
        LT = 2,
        RX = 3,
        RY = 4,
        RT = 5,
        HORIZONTAL = 6,
        VERTICAL = 7,
    };

    enum Button {
        A = 0,
        B = 1,
        X = 2,
        Y = 3,
        LB = 4,
        RB = 5,
        SELECT = 6,
        START = 7,
        HOME = 8,
        LSB = 9,
        RSB = 10,
    };

    JoystickWidget(QWidget *parent = nullptr);
    ~JoystickWidget();
    double get(const Analog &analog) const;
    bool get(const Button &button) const;
};
