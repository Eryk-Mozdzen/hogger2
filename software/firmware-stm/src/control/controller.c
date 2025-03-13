#include <stm32u5xx_hal.h>

#include "actuate/motors.h"
#include "actuate/servos.h"
#include "com/stream.h"
#include "utils/task.h"

static uint32_t last = 0;

static void receiver(mpack_t *mpack) {
    float reference[6];
    if(mpack_read_array(mpack, reference, 6)) {
        last = HAL_GetTick();

        motors_set_velocity(reference[2], reference[5]);
        servos_set_position(reference[0], reference[1], reference[3], reference[4]);
    }
}

static void shutdown(mpack_t *mpack) {
    (void) mpack;

    last = HAL_GetTick();

    motors_set_velocity(0, 0);
    servos_set_position(0, 0, 0, 0);
}

static void watchdog() {
    const uint32_t time = HAL_GetTick();

    if((time - last) >= 1000) {
        last = time;
        shutdown(NULL);
    }
}

STREAM_REGISTER("reference", receiver)
STREAM_REGISTER("stop", shutdown)
TASK_REGISTER_PERIODIC(watchdog, 1000)
