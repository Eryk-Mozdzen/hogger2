#include "actuate/motors.h"
#include "actuate/servos.h"
#include "com/stream.h"
#include "utils/task.h"

static uint32_t last = 0;

static void manual(mpack_t *mpack) {
    float manual[6];
    if(mpack_read_array(mpack, manual, 6)) {
        last = task_timebase();

        motors_set_velocity(manual[2], manual[5]);
        servos_set_position(manual[0], manual[1], manual[3], manual[4]);
    }
}

static void shutdown(mpack_t *mpack) {
    (void)mpack;

    last = task_timebase();

    motors_set_velocity(0, 0);
    servos_set_position(0, 0, 0, 0);
}

static void watchdog() {
    const uint32_t time = task_timebase();

    if((time - last) >= 1000000) {
        last = time;
        shutdown(NULL);
    }
}

STREAM_REGISTER("manual", manual)
STREAM_REGISTER("stop", shutdown)
TASK_REGISTER_PERIODIC(watchdog, 1000)
