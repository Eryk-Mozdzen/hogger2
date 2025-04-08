#include "actuate/motors.h"
#include "actuate/servos.h"
#include "com/stream.h"
#include "control/watchdog.h"

static void manual(mpack_t *mpack) {
    float manual[6];
    if(mpack_read_array(mpack, manual, 6)) {
        watchdog_reset();

        motors_set_velocity(manual[2], manual[5]);
        servos_set_position(manual[0], manual[1], manual[3], manual[4]);
    }
}

STREAM_REGISTER("manual", manual)
