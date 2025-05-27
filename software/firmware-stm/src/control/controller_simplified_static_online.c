#include <math.h>
#include <string.h>

#include "actuate/motors.h"
#include "actuate/servos.h"
#include "com/stream.h"
#include "com/telemetry.h"
#include "control/integrator.h"
#include "control/robot_parameters.h"
#include "control/simplified_static_online.h"
#include "control/trajectory.h"
#include "control/watchdog.h"
#include "generated/estimator.h"
#include "utils/task.h"

#define CONTROLLER_K 0.5f
#define GIMBAL_MAX   (2.f * M_PI / 180.f)

typedef enum {
    INTEGRAL_IDX_PHI1,
    INTEGRAL_IDX_THETA1,
    INTEGRAL_IDX_PHI2,
    INTEGRAL_IDX_THETA2,
} integral_idx_t;

typedef struct {
    bool started;
    uint32_t time_start;
    float time;
    float q[9];
    float hd[10];
} controller_t;

static const float K[25] = {
    CONTROLLER_K, 0, 0, 0, 0, 0, CONTROLLER_K, 0, 0, 0, 0, 0, CONTROLLER_K * 10, 0, 0, 0, 0, 0,
    CONTROLLER_K, 0, 0, 0, 0, 0, CONTROLLER_K,
};

static integrator_element_t integrator_elements[4];
static integrator_t integrator;
static controller_t controller = {0};

static void ok(mpack_t *mpack) {
    (void)mpack;

    watchdog_reset();

    if(!controller.started) {
        controller.started = true;
        controller.time_start = task_timebase();

        float phi1;
        float theta1;
        float phi2;
        float theta2;
        servos_get_position(&phi1, &theta1, &phi2, &theta2);

        integrator_reset(&integrator);
        integrator_set(&integrator, INTEGRAL_IDX_PHI1, phi1);
        integrator_set(&integrator, INTEGRAL_IDX_THETA1, theta1);
        integrator_set(&integrator, INTEGRAL_IDX_PHI2, phi2);
        integrator_set(&integrator, INTEGRAL_IDX_THETA2, theta2);

        servos_set_position(0.38f, 0.15f, 0.38f, 0.15);
    }
}

static void abort() {
    controller.started = false;
}

static void init() {
    integrator_init(&integrator, integrator_elements, 4, -GIMBAL_MAX, +GIMBAL_MAX);
}

static void loop() {
    const uint32_t now = task_timebase();

    controller.time = (now - controller.time_start) * 0.000001f;

    if(!controller.started) {
        memset(&controller, 0, sizeof(controller));
        return;
    }

    float phi1;
    float theta1;
    float phi2;
    float theta2;
    servos_get_position(&phi1, &theta1, &phi2, &theta2);

    controller.q[0] = estimator_state_get_px();
    controller.q[1] = estimator_state_get_py();
    controller.q[2] = estimator_state_get_theta();
    controller.q[3] = phi1;
    controller.q[4] = theta1;
    controller.q[5] = 0; // not used
    controller.q[6] = phi2;
    controller.q[7] = theta2;
    controller.q[8] = 0; // not used

    trajectory_t trajectory;
    trajectory_get(&trajectory, controller.time);

    const float sin_theta_ref = sinf(TRAJECTORY_GET_THETA(&trajectory));
    const float cos_theta_ref = cosf(TRAJECTORY_GET_THETA(&trajectory));

    controller.hd[0] = TRAJECTORY_GET_X(&trajectory) + ROBOT_PARAMETER_D * cos_theta_ref;
    controller.hd[1] = TRAJECTORY_GET_Y(&trajectory) + ROBOT_PARAMETER_D * sin_theta_ref;
    controller.hd[2] = 0.4f;
    controller.hd[3] = 0.02f;
    controller.hd[4] = 0.02f;
    controller.hd[5] = TRAJECTORY_GET_D_X(&trajectory) -
                       ROBOT_PARAMETER_D * TRAJECTORY_GET_D_THETA(&trajectory) * sin_theta_ref;
    controller.hd[6] = TRAJECTORY_GET_D_Y(&trajectory) +
                       ROBOT_PARAMETER_D * TRAJECTORY_GET_D_THETA(&trajectory) * cos_theta_ref;
    controller.hd[7] = 0;
    controller.hd[8] = 0;
    controller.hd[9] = 0;

    float eta[5];
    simplified_static_online_calculate(eta, K, controller.q, controller.hd);

    float input[4] = {0};
    input[INTEGRAL_IDX_PHI1] = eta[0];
    input[INTEGRAL_IDX_THETA1] = eta[1];
    input[INTEGRAL_IDX_PHI2] = eta[0] + (eta[2] * sinf(theta1)) - (eta[4] * sinf(theta2));
    input[INTEGRAL_IDX_THETA2] = eta[3];
    integrator_step(&integrator, input);

    servos_set_position(integrator_get(&integrator, INTEGRAL_IDX_PHI1),
                        integrator_get(&integrator, INTEGRAL_IDX_THETA1),
                        integrator_get(&integrator, INTEGRAL_IDX_PHI2),
                        integrator_get(&integrator, INTEGRAL_IDX_THETA2));
}

static void serialize(cmp_ctx_t *cmp, void *context) {
    (void)context;

    cmp_write_map(cmp, 2);
    cmp_write_str(cmp, "started", 7);
    cmp_write_bool(cmp, controller.started);
    cmp_write_str(cmp, "time", 4);
    cmp_write_bool(cmp, controller.time);
}

STREAM_REGISTER("controller_continue", ok)
WATCHDOG_REGISTER(abort)
TASK_REGISTER_INIT(init)
TASK_REGISTER_PERIODIC(loop, 1000)
TELEMETRY_REGISTER("controller", serialize, NULL)
