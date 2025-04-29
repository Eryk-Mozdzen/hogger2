#include "actuate/motors.h"
#include "actuate/servos.h"
#include "com/stream.h"
#include "control/naive.h"
#include "control/trajectory.h"
#include "control/watchdog.h"
#include "generated/estimator.h"
#include "utils/task.h"

#define CONTROLLER_K 1.f
#define MOTOR_VEL    300.f
#define GIMBAL_MAX   (5.f * M_PI / 180.f)

#define CLAMP(val, min, max) (((val) > (max)) ? (max) : (((val) < (min)) ? (min) : (val)))

typedef struct {
    bool started;
    uint32_t time_start;
    float time;
    float q[3];
    float ref[4];
} controller_t;

static controller_t controller = {0};

static void ok(mpack_t *mpack) {
    (void)mpack;

    watchdog_reset();

    if(!controller.started) {
        controller.started = true;
        controller.time_start = task_timebase();
    }
}

static void abort() {
    controller.started = false;
}

static void loop() {
    const uint32_t now = task_timebase();

    controller.time = (now - controller.time_start) * 0.000001f;

    if(!controller.started) {
        memset(&controller, 0, sizeof(controller));
        return;
    }

    trajectory_t trajectory;
    trajectory_get(&trajectory, controller.time);

    controller.ref[0] = TRAJECTORY_GET_X(&trajectory);
    controller.ref[1] = TRAJECTORY_GET_Y(&trajectory);
    controller.ref[2] = TRAJECTORY_GET_D_X(&trajectory);
    controller.ref[3] = TRAJECTORY_GET_D_Y(&trajectory);

    controller.q[0] = ESTIMATOR_GET_POS_X();
    controller.q[1] = ESTIMATOR_GET_POS_Y();
    controller.q[2] = ESTIMATOR_GET_POS_THETA();

    float gimbal[2];
    naive_feedback(gimbal, MOTOR_VEL, controller.q, controller.ref, CONTROLLER_K);

    const float phi12d = CLAMP(gimbal[0], -GIMBAL_MAX, +GIMBAL_MAX);
    const float theta12d = CLAMP(gimbal[1], -GIMBAL_MAX, +GIMBAL_MAX);

    servos_set_position(phi12d, theta12d, phi12d, theta12d);
}

STREAM_REGISTER("controller_continue", ok)
WATCHDOG_REGISTER(abort)
TASK_REGISTER_PERIODIC(loop, 10000)
