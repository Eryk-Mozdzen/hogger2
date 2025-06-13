#include <math.h>

#include "actuate/servos.h"
#include "com/stream.h"
#include "com/telemetry.h"
#include "control/pid.h"
#include "control/robot_parameters.h"
#include "control/trajectory.h"
#include "control/watchdog.h"
#include "generated/estimator.h"
#include "utils/task.h"

#define MOTOR_VEL -250.f

// #define EXPERIMENT_INNER_THETA
// #define EXPERIMENT_OUTER_THETA
// #define EXPERIMENT_INNER_X
// #define EXPERIMENT_INNER_Y
// #define EXPERIMENT_OUTER_X
// #define EXPERIMENT_OUTER_Y

typedef struct {
    bool started;
    uint32_t time_start;
    float time;

    float exp_step;
    float exp_response;

    pid_t inner_x;
    pid_t inner_y;
    pid_t inner_theta;

    pid_t outer_x;
    pid_t outer_y;
    pid_t outer_theta;
} controller_t;

static controller_t controller = {
    .inner_theta = PID_INIT(2.260574, 3.511004, 0, -4 * M_PI, 4 * M_PI), // PI CHR 0%
    .outer_theta = PID_INIT(11.334450, 0, 3.838592, -M_PI, M_PI),        // PD Lambda = 0.25
    .inner_x = PID_INIT(3.726573, 5.949067, 0, -1.5, 1.5),               // PI CHR 0%
    .inner_y = PID_INIT(0.816472, 1.703120, 0, -1.5, 1.5),               // PI CHR 0%
    .outer_x = PID_INIT(4.257331, 0, 1.104396, -0.5, 0.5),               // PD Lambda = 1
    .outer_y = PID_INIT(3.626742, 0, 1.073429, -0.5, 0.5),               // PD Lambda = 1
};

static void Tu(const float u_x, const float u_y, const float u_theta, float *output) {
    output[0] = -u_x / (ROBOT_PARAMETER_R * MOTOR_VEL);
    output[1] = -u_y / (ROBOT_PARAMETER_R * MOTOR_VEL);
    output[2] = (u_x / (ROBOT_PARAMETER_R * MOTOR_VEL)) +
                ((2 * ROBOT_PARAMETER_L * u_theta) / (ROBOT_PARAMETER_R * MOTOR_VEL));
    output[3] = u_y / (ROBOT_PARAMETER_R * MOTOR_VEL);
}

static void Ttheta(const float alpha, const float x, const float y, float *output) {
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
    const float dtheta = estimator_state_get_dottheta();

    float local_pos_ref[2];
    Ttheta(-theta, TRAJECTORY_GET_X(&trajectory), TRAJECTORY_GET_Y(&trajectory), local_pos_ref);

    float local_vel_ref[2];
    Ttheta(-theta, TRAJECTORY_GET_D_X(&trajectory), TRAJECTORY_GET_D_Y(&trajectory), local_vel_ref);

    float local_pos[2];
    Ttheta(-theta, estimator_state_get_x(), estimator_state_get_y(), local_pos);

    float local_vel[2];
    Ttheta(-theta, estimator_state_get_dotx(), estimator_state_get_doty(), local_vel);

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
    const float u_theta = pid_calculate(&controller.inner_theta, inner_vel_ref_theta, dtheta);
#endif

#ifdef EXPERIMENT_INNER_THETA
    controller.exp_step = (controller.time > 3) ? 2 : 0;
    controller.exp_response = dtheta;

    const float u_x = 0;
    const float u_y = 0;
    const float u_theta = controller.exp_step;
#endif

#ifdef EXPERIMENT_OUTER_THETA
    controller.exp_step = (controller.time > 3) ? 2 : 0;
    controller.exp_response = theta;

    const float u_x = 0;
    const float u_y = 0;
    const float u_theta = pid_calculate(&controller.inner_theta, controller.exp_step, dtheta);
#endif

#ifdef EXPERIMENT_INNER_X
    controller.exp_step = (controller.time > 3) ? 0.5f : 0;
    controller.exp_response = local_vel[0];

    const float vel_theta = pid_calculate(&controller.outer_theta, 0, theta);

    const float u_x = controller.exp_step;
    const float u_y = 0;
    const float u_theta = pid_calculate(&controller.inner_theta, vel_theta, dtheta);
#endif

#ifdef EXPERIMENT_INNER_Y
    controller.exp_step = (controller.time > 3) ? 0.5f : 0;
    controller.exp_response = local_vel[1];

    const float vel_theta = pid_calculate(&controller.outer_theta, 0, theta);

    const float u_x = 0;
    const float u_y = controller.exp_step;
    const float u_theta = pid_calculate(&controller.inner_theta, vel_theta, dtheta);
#endif

#ifdef EXPERIMENT_OUTER_X
    controller.exp_step = (controller.time > 5) ? 0.5f : 0;
    controller.exp_response = local_pos[0];

    const float vel_theta = pid_calculate(&controller.outer_theta, 0, theta);

    const float u_x = pid_calculate(&controller.inner_x, controller.exp_step, local_vel[0]);
    const float u_y = pid_calculate(&controller.inner_y, 0, local_vel[1]);
    const float u_theta = pid_calculate(&controller.inner_theta, vel_theta, dtheta);
#endif

#ifdef EXPERIMENT_OUTER_Y
    controller.exp_step = (controller.time > 5) ? 0.5f : 0;
    controller.exp_response = local_pos[1];

    const float vel_theta = pid_calculate(&controller.outer_theta, 0, theta);

    const float u_x = pid_calculate(&controller.inner_x, 0, local_vel[0]);
    const float u_y = pid_calculate(&controller.inner_y, controller.exp_step, local_vel[1]);
    const float u_theta = pid_calculate(&controller.inner_theta, vel_theta, dtheta);
#endif

    float angles[4];
    Tu(u_x, u_y, u_theta, angles);

    servos_set_position(angles[0], angles[1], angles[2], angles[3]);
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
