#include <main.h>
#include <stm32h5xx_hal.h>

#include "com/stream.h"

static void blink(mpack_t *mpack) {
    bool state;
    if(mpack_read_bool(mpack, &state)) {
        HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, state);
    }
}

STREAM_REGISTER("blink", blink);
