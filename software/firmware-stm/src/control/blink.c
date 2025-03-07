#include <main.h>
#include <stm32u5xx_hal.h>

#include "actuate/servos.h"
#include "com/stream.h"

static void blink(mpack_t *mpack) {
    bool state;
    if(mpack_read_bool(mpack, &state)) {
        HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, state);
        servos_set_led(state, state, state, state);
    }
}

STREAM_REGISTER("blink", blink);
