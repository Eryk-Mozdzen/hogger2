#include <math.h>
#include <string.h>

#include "actuate/servos.h"
#include "com/stream.h"
#include "com/telemetry.h"
#include "control/derivative.h"
#include "control/integrator.h"
#include "control/naive.h"
#include "control/trajectory.h"
#include "control/watchdog.h"
#include "generated/estimator.h"
#include "utils/task.h"

#define MOTOR_VEL 200.f

typedef struct {
    float kp;
    float ki;
    float kd;
    integrator_element_t integrator_element;
    integrator_t integrator;
    derivative_element_t derivative_element;
    derivative_t derivative;
} pid_t;

typedef struct {
    bool started;
    uint32_t time_start;
    float time;

    pid_t pid_x;
    pid_t pid_y;
    pid_t pid_theta;
} controller_t;

static controller_t controller = {0};

static void pid_init(pid_t *pid, const float kp, const float ki, const float kd) {
    pid->kp = kp;
    pid->ki = ki;
    pid->kd = kd;
    integrator_init(&pid->integrator, &pid->integrator_element, 1, -1, 1);
    derivative_init(&pid->derivative, &pid->derivative_element, 1, -10, 10);
}

static float pid_calculate(pid_t *pid, const float sp, const float pv) {
    const float e = sp - pv;

    integrator_step(&pid->integrator, &e);
    derivative_step(&pid->derivative, &e);

    const float ie = integrator_get(&pid->integrator, 0);
    const float de = derivative_get(&pid->derivative, 0);

    return pid->kp * e + pid->ki * ie + pid->kd * de;
}

static void ok(mpack_t *mpack) {
    (void)mpack;

    watchdog_reset();

    if(!controller.started) {
        controller.started = true;
        controller.time_start = task_timebase();

        pid_init(&controller.pid_x, 0.099, 0.24, 0); // Reswick 0%
        pid_init(&controller.pid_y, 0.099, 0.24, 0);
        pid_init(&controller.pid_theta, 0.029, 0.069, 0); // Reswick 0%
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

    const float k = 2;

    const float sp_dx = TRAJECTORY_GET_D_X(&trajectory) -
                        k * (estimator_state_get_px() - TRAJECTORY_GET_X(&trajectory));
    const float sp_dy = TRAJECTORY_GET_D_Y(&trajectory) -
                        k * (estimator_state_get_py() - TRAJECTORY_GET_Y(&trajectory));
    const float sp_dtheta = TRAJECTORY_GET_D_THETA(&trajectory) -
                            k * (estimator_state_get_theta() - TRAJECTORY_GET_THETA(&trajectory));

    const float thetaxxx = estimator_state_get_theta();
    const float local_dx =
        +estimator_state_get_vx() * cosf(thetaxxx) + estimator_state_get_vy() * sinf(thetaxxx);
    const float local_dy =
        -estimator_state_get_vx() * sinf(thetaxxx) + estimator_state_get_vy() * cosf(thetaxxx);

    const float local_target_dx = +sp_dx * cosf(thetaxxx) + sp_dy * sinf(thetaxxx);
    const float local_target_dy = -sp_dx * sinf(thetaxxx) + sp_dy * cosf(thetaxxx);

    const float x = pid_calculate(&controller.pid_x, local_target_dx, local_dx);
    const float y = pid_calculate(&controller.pid_y, local_target_dy, local_dy);
    const float theta =
        pid_calculate(&controller.pid_theta, sp_dtheta, estimator_state_get_vtheta());

    const float phi1 = x;
    const float theta12 = y;
    const float phi2 = x + theta;

    servos_set_position(+phi1, +theta12, -phi2, -theta12);
}

static void serialize(cmp_ctx_t *cmp, void *context) {
    (void)context;

    cmp_write_map(cmp, 1);
    cmp_write_str(cmp, "started", 7);
    cmp_write_bool(cmp, controller.started);
}

STREAM_REGISTER("controller_continue", ok)
WATCHDOG_REGISTER(abort)
TASK_REGISTER_PERIODIC(loop, 1000)
TELEMETRY_REGISTER("controller", serialize, NULL)
