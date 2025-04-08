#include "control/watchdog.h"
#include "actuate/motors.h"
#include "actuate/servos.h"
#include "com/stream.h"
#include "utils/task.h"

#define MAX 4

static watchdog_callback_t registered[MAX] = {0};
static uint32_t count = 0;
static uint32_t last = 0;

static void shutdown(mpack_t *mpack) {
    (void)mpack;

    motors_set_velocity(0, 0);
    servos_set_position(0, 0, 0, 0);
}

static void watchdog() {
    const uint32_t time = task_timebase();

    if((time - last) >= 1000000) {
        last = time;

        for(uint32_t i = 0; i < count; i++) {
            if(registered[i]) {
                registered[i]();
            }
        }

        shutdown(NULL);
    }
}

void watchdog_register(const watchdog_callback_t callback) {
    registered[count] = callback;
    count++;
}

void watchdog_reset() {
    last = task_timebase();
}

STREAM_REGISTER("stop", shutdown)
TASK_REGISTER_PERIODIC(watchdog, 1000)
