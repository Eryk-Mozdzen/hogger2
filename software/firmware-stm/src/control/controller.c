#include <stm32u5xx_hal.h>

#include "actuate/motors.h"
#include "com/stream.h"
#include "utils/tasks.h"
// servos

static uint32_t last = 0;

static void receiver(mpack_t *mpack) {
    if(!mpack_read_map(mpack, 1)) {
        return;
    }

    if(!mpack_read_str(mpack, "ref_cfg")) {
        return;
    }

    float reference[6];
    if(mpack_read_array(mpack, reference, 6)) {
        last = HAL_GetTick();

        motors_set_velocity(reference[2], reference[5]);
        // servos
    }
}

static void shutdown() {
    last = HAL_GetTick();

    motors_set_velocity(0, 0);
    // servos
}

static void watchdog() {
    const uint32_t time = HAL_GetTick();

    if((time - last)>=1000) {
        last = time;
        shutdown();
    }
}

STREAM_REGISTER("reference", receiver)
STREAM_REGISTER("stop", shutdown)
TASKS_REGISTER(NULL, watchdog, 1);
