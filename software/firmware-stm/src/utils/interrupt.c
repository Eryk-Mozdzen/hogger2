#include <stdint.h>

#include "interrupt.h"

#define MAX 8

typedef struct {
    interrupt_callback_t callback;
    uint16_t pin;
} register_t;

static register_t registered[MAX] = {0};
static uint32_t count = 0;

void interrupt_register(const interrupt_callback_t callback, const uint16_t pin) {
    registered[count].callback = callback;
    registered[count].pin = pin;
    count++;
}

void HAL_GPIO_EXTI_Rising_Callback(uint16_t GPIO_Pin) {
    for(uint32_t i = 0; i < count; i++) {
        if((GPIO_Pin == registered[i].pin) && registered[i].callback) {
            registered[i].callback();
        }
    }
}
