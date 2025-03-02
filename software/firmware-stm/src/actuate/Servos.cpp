#include <stm32u5xx_hal.h>

#include "actuate/DynamixelBus.hpp"
#include "actuate/DynamixelServo.hpp"
#include "freertos/Task.hpp"

extern UART_HandleTypeDef huart4;

class Servos : freertos::TaskClass<1024> {
    void task();

public:
    DynamixelBus bus;
    DynamixelServo servo1x;
    DynamixelServo servo1y;
    DynamixelServo servo2x;
    DynamixelServo servo2y;

    Servos();
};

Servos servos;

static void transmitISR(UART_HandleTypeDef *huart) {
    servos.bus.transmitISR(huart);
}

Servos::Servos()
    : TaskClass{"servos", 14},
      bus{&huart4},
      servo1x{"servo_1_x", bus, 0x00},
      servo1y{"servo_1_y", bus, 0x01},
      servo2x{"servo_2_x", bus, 0x02},
      servo2y{"servo_2_y", bus, 0x03} {
}

void Servos::task() {
    uint32_t time = 0;

    HAL_UART_RegisterCallback(&huart4, HAL_UART_TX_COMPLETE_CB_ID, transmitISR);

    while(true) {
        bus.tick();

        // TODO: transmit position and velocity to measurement queue

        xTaskDelayUntil(&time, 20);
    }
}

namespace servo {

void setGoal(const float x1, const float y1, const float x2, const float y2) {
    servos.servo1x.setGoal(x1);
    servos.servo1y.setGoal(y1);
    servos.servo2x.setGoal(x2);
    servos.servo2y.setGoal(y2);
}

void setLed(const bool x1, const bool y1, const bool x2, const bool y2) {
    servos.servo1x.setLed(x1);
    servos.servo1y.setLed(y1);
    servos.servo2x.setLed(x2);
    servos.servo2y.setLed(y2);
}

}
