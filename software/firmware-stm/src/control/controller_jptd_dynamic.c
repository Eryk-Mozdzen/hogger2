#include <math.h>

#include "actuate/motors.h"
#include "actuate/servos.h"
#include "com/stream.h"
#include "com/telemetry.h"
#include "control/integrator.h"
#include "control/jptd_dynamic.h"
#include "control/trajectory.h"
#include "control/watchdog.h"
#include "generated/estimator.h"
#include "utils/task.h"

#define CONTROLLER_K1 5.f
#define CONTROLLER_K2 10.f
#define MOTOR1_VEL    -300.f
#define MOTOR2_VEL    +300.f
#define GIMBAL_MAX    (2.5f * M_PI / 180.f)

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
    float h[10];
    float hd[15];
} controller_t;

static const float K1[25] = {
    CONTROLLER_K1, 0, 0, 0, 0,
    0, CONTROLLER_K1, 0, 0, 0,
    0, 0, CONTROLLER_K1, 0, 0,
    0, 0, 0, CONTROLLER_K1, 0,
    0, 0, 0, 0, CONTROLLER_K1,
};

static const float K2[25] = {
    CONTROLLER_K2, 0, 0, 0, 0,
    0, CONTROLLER_K2, 0, 0, 0,
    0, 0, CONTROLLER_K2, 0, 0,
    0, 0, 0, CONTROLLER_K2, 0,
    0, 0, 0, 0, CONTROLLER_K2,
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

        servos_set_position(+0.0472104532, -0.0471579290, -0.0472104532, +0.0471579290);
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

    controller.h[0] = ESTIMATOR_GET_POS_X();
    controller.h[1] = ESTIMATOR_GET_POS_Y();
    controller.h[2] = ESTIMATOR_GET_POS_THETA();
    controller.h[3] = MOTOR1_VEL * controller.time;
    controller.h[4] = MOTOR2_VEL * controller.time;
    controller.h[5] = ESTIMATOR_GET_VEL_X();
    controller.h[6] = ESTIMATOR_GET_VEL_Y();
    controller.h[7] = ESTIMATOR_GET_VEL_THETA();
    controller.h[8] = MOTOR1_VEL;
    controller.h[9] = MOTOR2_VEL;

    trajectory_t trajectory;
    trajectory_get(&trajectory, controller.time);

    controller.hd[0] = TRAJECTORY_GET_X(&trajectory);
    controller.hd[1] = TRAJECTORY_GET_Y(&trajectory);
    controller.hd[2] = TRAJECTORY_GET_THETA(&trajectory) + M_PI/4;
    controller.hd[3] = MOTOR1_VEL * controller.time;
    controller.hd[4] = MOTOR2_VEL * controller.time;
    controller.hd[5] = TRAJECTORY_GET_D_X(&trajectory);
    controller.hd[6] = TRAJECTORY_GET_D_Y(&trajectory);
    controller.hd[7] = TRAJECTORY_GET_D_THETA(&trajectory);
    controller.hd[8] = MOTOR1_VEL;
    controller.hd[9] = MOTOR2_VEL;
    controller.hd[10] = TRAJECTORY_GET_D2_X(&trajectory);
    controller.hd[11] = TRAJECTORY_GET_D2_Y(&trajectory);
    controller.hd[12] = TRAJECTORY_GET_D2_THETA(&trajectory);
    controller.hd[13] = 0;
    controller.hd[14] = 0;

    float v[5];
    jptd_dynamic_feedback_v(v, K1, K2, controller.h, controller.hd);

    float phi1;
    float theta1;
    float phi2;
    float theta2;
    servos_get_position(&phi1, &theta1, &phi2, &theta2);

    controller.q[0] = ESTIMATOR_GET_POS_X();
    controller.q[1] = ESTIMATOR_GET_POS_Y();
    controller.q[2] = ESTIMATOR_GET_POS_THETA();
    controller.q[3] = phi1;
    controller.q[4] = theta1;
    controller.q[5] = MOTOR1_VEL * controller.time;
    controller.q[6] = phi2;
    controller.q[7] = theta2;
    controller.q[8] = MOTOR2_VEL * controller.time;

    float eta[5] = {
        0, 0, MOTOR1_VEL, 0, MOTOR2_VEL,
    };

    float u[5];
    jptd_dynamic_linearize_u(u, v, eta, controller.q);

    eta[0] = u[0];
    eta[1] = u[1];
    eta[3] = u[3];

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
