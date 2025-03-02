#include <main.h>
#include <stm32u5xx_hal.h>

#include "actuate/Servos.hpp"
#include "com/Mailbox.hpp"
#include "freertos/Task.hpp"

class Blink : freertos::TaskClass<128>, Mailbox<16> {
    void task();

public:
    Blink();
};

Blink blink;

Blink::Blink() : TaskClass{"blink", 1}, Mailbox{1} {
}

void Blink::task() {
    uint8_t state;

    while(true) {
        xMessageBufferReceive(getBuffer(), &state, sizeof(state), portMAX_DELAY);

        HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, state ? GPIO_PIN_SET : GPIO_PIN_RESET);

        servo::setLed(state, state, state, state);
    }
}
