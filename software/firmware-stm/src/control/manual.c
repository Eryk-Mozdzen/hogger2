#include "actuate/motors.h"
#include "actuate/servos.h"
#include "com/stream.h"
#include "control/watchdog.h"

static void servo(mpack_t *mpack) {
    float manual[4];
    if(mpack_read_float32_array(mpack, manual, 4, NULL)) {
        watchdog_reset();

        servos_set_position(manual[0], manual[1], manual[2], manual[3]);
    }
}

static void motor(mpack_t *mpack) {
    float manual[2];
    if(mpack_read_float32_array(mpack, manual, 2, NULL)) {
        watchdog_reset();

        motors_set_velocity(manual[0], manual[1]);
    }
}

STREAM_REGISTER("manual_servo", servo)
STREAM_REGISTER("manual_motor", motor)
