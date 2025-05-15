#include <math.h>

#include "actuate/servos.h"
#include "com/stream.h"
#include "com/telemetry.h"
#include "control/moving_average.h"
#include "control/pid.h"
#include "control/trajectory.h"
#include "control/watchdog.h"
#include "generated/estimator.h"
#include "utils/task.h"

#define DEG2RAD(deg) (M_PI * (deg) / 180.f)

// #define EXPERIMENT_INNER_THETA
// #define EXPERIMENT_INNER_X
// #define EXPERIMENT_INNER_Y
// #define EXPERIMENT_OUTER_THETA
// #define EXPERIMENT_OUTER_X
// #define EXPERIMENT_OUTER_Y

typedef struct {
    bool started;
    uint32_t time_start;
    float time;

    float exp_step;
    float exp_response;

    float filter_vel_x_buffer[50];
    float filter_vel_y_buffer[50];

    moving_average_t filter_vel_x;
    moving_average_t filter_vel_y;

    pid_t inner_x;
    pid_t inner_y;
    pid_t inner_theta;

    pid_t outer_x;
    pid_t outer_y;
    pid_t outer_theta;
} controller_t;

static controller_t controller = {
    .filter_vel_x = MOVING_AVERAGE_INIT(controller.filter_vel_x_buffer, 50),
    .filter_vel_y = MOVING_AVERAGE_INIT(controller.filter_vel_y_buffer, 50),

    .inner_x = PID_INIT(0.112, 0.117, 0, DEG2RAD(-4), DEG2RAD(4)),     // CHR 0%
    .inner_y = PID_INIT(0.029, 0.061, 0, DEG2RAD(-4), DEG2RAD(4)),     // CHR 0%
    .inner_theta = PID_INIT(0.027, 0.065, 0, DEG2RAD(-4), DEG2RAD(4)), // CHR 0%

    .outer_x = PID_INIT(0.980, 0.408, 0, -0.5, 0.5),               // Lambda = 1
    .outer_y = PID_INIT(2.449, 1.020, 0, -0.5, 0.5),               // Lambda = 1
    .outer_theta = PID_INIT(2.963, 2.963, 0, -2 * M_PI, 2 * M_PI), // Lambda = 0.25
};

static void rot2d(const float alpha, const float x, const float y, float *output) {
    const float sin_alpha = sinf(alpha);
    const float cos_alpha = cosf(alpha);

    output[0] = (x * cos_alpha) - (y * sin_alpha);
    output[1] = (x * sin_alpha) + (y * cos_alpha);
}

static void ok(mpack_t *mpack) {
    (void)mpack;

    watchdog_reset();

    if(!controller.started) {
        controller.started = true;
        controller.time_start = task_timebase();

        moving_average_reset(&controller.filter_vel_x);
        moving_average_reset(&controller.filter_vel_y);

        pid_reset(&controller.inner_x);
        pid_reset(&controller.inner_y);
        pid_reset(&controller.inner_theta);

        pid_reset(&controller.outer_x);
        pid_reset(&controller.outer_y);
        pid_reset(&controller.outer_theta);
    }
}

static void abort() {
    controller.started = false;
}

static void loop() {
    const uint32_t now = task_timebase();

    controller.time = (now - controller.time_start) * 0.000001f;

    if(!controller.started) {
        return;
    }

    trajectory_t trajectory;
    trajectory_get(&trajectory, controller.time);

    const float theta = estimator_state_get_theta();

    float local_pos_ref[2];
    rot2d(-theta, TRAJECTORY_GET_X(&trajectory), TRAJECTORY_GET_Y(&trajectory), local_pos_ref);

    float local_vel_ref[2];
    rot2d(-theta, TRAJECTORY_GET_D_X(&trajectory), TRAJECTORY_GET_D_Y(&trajectory), local_vel_ref);

    float local_pos[2];
    rot2d(-theta, estimator_state_get_px(), estimator_state_get_py(), local_pos);

    float local_vel[2];
    rot2d(-theta, estimator_state_get_vx(), estimator_state_get_vy(), local_vel);

    local_vel[0] = moving_average_append(&controller.filter_vel_x, local_vel[0]);
    local_vel[1] = moving_average_append(&controller.filter_vel_y, local_vel[1]);

#if !defined(EXPERIMENT_INNER_THETA) && !defined(EXPERIMENT_INNER_X) &&                            \
    !defined(EXPERIMENT_INNER_Y) && !defined(EXPERIMENT_OUTER_THETA) &&                            \
    !defined(EXPERIMENT_OUTER_X) && !defined(EXPERIMENT_OUTER_Y)
    const float vel_x = pid_calculate(&controller.outer_x, local_pos_ref[0], local_pos[0]);
    const float vel_y = pid_calculate(&controller.outer_y, local_pos_ref[1], local_pos[1]);
    const float vel_theta =
        pid_calculate(&controller.outer_theta, TRAJECTORY_GET_THETA(&trajectory), theta);

    const float inner_vel_ref_x = local_vel_ref[0] + vel_x;
    const float inner_vel_ref_y = local_vel_ref[1] + vel_y;
    const float inner_vel_ref_theta = TRAJECTORY_GET_D_THETA(&trajectory) + vel_theta;

    const float u_x = pid_calculate(&controller.inner_x, inner_vel_ref_x, local_vel[0]);
    const float u_y = pid_calculate(&controller.inner_y, inner_vel_ref_y, local_vel[1]);
    const float u_theta =
        pid_calculate(&controller.inner_theta, inner_vel_ref_theta, estimator_state_get_vtheta());
#endif

#ifdef EXPERIMENT_INNER_THETA
    controller.exp_step = (controller.time > 3) ? 0.05f : 0;
    controller.exp_response = estimator_state_get_vtheta();

    const float u_x = 0;
    const float u_y = 0;
    const float u_theta = controller.exp_step;
#endif

#ifdef EXPERIMENT_INNER_X
    controller.exp_step = (controller.time > 3) ? 0.1f : 0;
    controller.exp_response = local_vel[0];

    const float u_x = controller.exp_step;
    const float u_y = 0;
    const float u_theta = pid_calculate(&controller.inner_theta, 0, estimator_state_get_vtheta());
#endif

#ifdef EXPERIMENT_INNER_Y
    controller.exp_step = (controller.time > 3) ? 0.05f : 0;
    controller.exp_response = local_vel[1];

    const float u_x = 0;
    const float u_y = controller.exp_step;
    const float u_theta = pid_calculate(&controller.inner_theta, 0, estimator_state_get_vtheta());
#endif

#ifdef EXPERIMENT_OUTER_THETA
    controller.exp_step = (controller.time > 3) ? 1 : 0;
    controller.exp_response = theta;

    const float u_x = pid_calculate(&controller.inner_x, 0, local_vel[0]);
    const float u_y = pid_calculate(&controller.inner_y, 0, local_vel[1]);
    const float u_theta =
        pid_calculate(&controller.inner_theta, controller.exp_step, estimator_state_get_vtheta());
#endif

#ifdef EXPERIMENT_OUTER_X
    controller.exp_step = (controller.time > 5) ? 0.5f : 0;
    controller.exp_response = local_pos[0];

    const float vel_theta = pid_calculate(&controller.outer_theta, 0, theta);

    const float u_x = pid_calculate(&controller.inner_x, controller.exp_step, local_vel[0]);
    const float u_y = pid_calculate(&controller.inner_y, 0, local_vel[1]);
    const float u_theta =
        pid_calculate(&controller.inner_theta, vel_theta, estimator_state_get_vtheta());
#endif

#ifdef EXPERIMENT_OUTER_Y
    controller.exp_step = (controller.time > 5) ? 1.f : 0;
    controller.exp_response = local_pos[1];

    const float vel_theta = pid_calculate(&controller.outer_theta, 0, theta);

    const float u_x = pid_calculate(&controller.inner_x, 0, local_vel[0]);
    const float u_y = pid_calculate(&controller.inner_y, controller.exp_step, local_vel[1]);
    const float u_theta =
        pid_calculate(&controller.inner_theta, vel_theta, estimator_state_get_vtheta());
#endif

    const float phi1 = u_x;
    const float theta12 = u_y;
    const float phi2 = u_x + u_theta;

    servos_set_position(+phi1, +theta12, -phi2, -theta12);
}

static void serialize(cmp_ctx_t *cmp, void *context) {
    (void)context;

    cmp_write_map(cmp, 3);
    cmp_write_str(cmp, "started", 7);
    cmp_write_bool(cmp, controller.started);

    cmp_write_str(cmp, "exp_step", 8);
    cmp_write_float(cmp, controller.exp_step);

    cmp_write_str(cmp, "exp_response", 12);
    cmp_write_float(cmp, controller.exp_response);
}

STREAM_REGISTER("controller_continue", ok)
WATCHDOG_REGISTER(abort)
TASK_REGISTER_PERIODIC(loop, 1000)
TELEMETRY_REGISTER("controller", serialize, NULL)
