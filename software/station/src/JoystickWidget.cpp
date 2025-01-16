#include <QTimer>
#include <QGridLayout>
#include <QFormLayout>
#include <QGroupBox>
#include <QComboBox>
#include <QPushButton>
#include <QSettings>

#include "JoystickWidget.h"
#include "Joystick.h"

Q_DECLARE_METATYPE(utils::Joystick::Info)

JoystickWidget::JoystickWidget(QWidget *parent) : QGroupBox{"Joystick", parent} {
    QVBoxLayout *layout = new QVBoxLayout(this);

    addressComboBox = new QComboBox();
    connect(addressComboBox, &QComboBox::currentIndexChanged, this, &JoystickWidget::changeInput);

    QPushButton *scanButton = new QPushButton("Scan ports");
    connect(scanButton, &QPushButton::pressed, this, &JoystickWidget::scanInput);

    QPushButton *saveButton = new QPushButton("Save as default");
    connect(saveButton, &QPushButton::pressed, [this]() {
        settings.setValue("defaultInput", addressComboBox->currentText());
    });

    QTimer *timer = new QTimer();
    connect(timer, &QTimer::timeout, [this]() {
        if(!joystick) {
            return;
        }

        if(!joystick->isAvailable()) {
            delete joystick;
            joystick = nullptr;
            addressComboBox->clear();
            return;
        }
    });
    timer->start(100);

    layout->addWidget(addressComboBox);
    layout->addWidget(scanButton);
    layout->addWidget(saveButton);

    setLayout(layout);

    scanInput();
}

JoystickWidget::~JoystickWidget() {
    if(joystick) {
        delete joystick;
    }
}

void JoystickWidget::scanInput() {
    addressComboBox->clear();

    const auto available = utils::Joystick::getAvailable();

	for(const auto &info : available) {
        addressComboBox->addItem(info.name.c_str(), QVariant::fromValue(info));
	}

    const int index = addressComboBox->findText(settings.value("defaultInput").toString());

    if(index != -1) {
        addressComboBox->setCurrentIndex(index);
    }

    changeInput(addressComboBox->currentIndex());
}

void JoystickWidget::changeInput(const int index) {
    if(addressComboBox->count()==0) {
        return;
    }

    if(joystick) {
        delete joystick;
        joystick = nullptr;
    }

    const utils::Joystick::Info info = addressComboBox->itemData(index).value<utils::Joystick::Info>();

    joystick = new utils::Joystick(info);
}

double JoystickWidget::get(const Analog &analog) const {
    return joystick ? joystick->getAxis(analog) : 0;
}

bool JoystickWidget::get(const Button &button) const {
    return joystick ? joystick->getButton(button) : false;
}
