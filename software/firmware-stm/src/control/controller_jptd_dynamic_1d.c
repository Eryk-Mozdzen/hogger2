#include "actuate/motors.h"
#include "actuate/servos.h"
#include "com/stream.h"
#include "control/integrator.h"
#include "control/jptd_dynamic_1d.h"
#include "control/trajectory.h"
#include "control/watchdog.h"
#include "generated/estimator.h"
#include "utils/task.h"

#define CONTROLLER_K1 5.f
#define CONTROLLER_K2 10.f
#define MOTOR_VEL     -300.f
#define GIMBAL_MAX    (5.f * M_PI / 180.f)

typedef enum {
    INTEGRAL_IDX_PHI1,
    INTEGRAL_IDX_PHI2,
} integral_idx_t;

typedef struct {
    bool started;
    uint32_t time_start;
    float time;
    float q[6];
    float h[6];
    float hd[9];
} controller_t;

static const float K1[9] = {
    CONTROLLER_K1, 0, 0, 0, CONTROLLER_K1, 0, 0, 0, CONTROLLER_K1,
};

static const float K2[9] = {
    CONTROLLER_K2, 0, 0, 0, CONTROLLER_K2, 0, 0, 0, CONTROLLER_K2,
};

static integrator_element_t integrator_elements[2];
static integrator_t integrator;
static controller_t controller = {0};

static void ok(mpack_t *mpack) {
    (void)mpack;

    watchdog_reset();

    if(!controller.started) {
        controller.started = true;
        controller.time_start = task_timebase();

        float phi1;
        float phi2;
        servos_get_position(&phi1, NULL, &phi2, NULL);

        integrator_reset(&integrator);
        integrator_set(&integrator, INTEGRAL_IDX_PHI1, phi1);
        integrator_set(&integrator, INTEGRAL_IDX_PHI2, phi2);
    }
}

static void abort() {
    controller.started = false;
}

static void init() {
    integrator_init(&integrator, integrator_elements, 2, -GIMBAL_MAX, +GIMBAL_MAX);
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

    float phi1;
    float phi2;
    servos_get_position(&phi1, NULL, &phi2, NULL);

    controller.h[0] = ESTIMATOR_GET_POS_X();
    controller.h[1] = ESTIMATOR_GET_POS_Y();
    controller.h[2] = MOTOR_VEL * controller.time;
    controller.h[3] = ESTIMATOR_GET_VEL_X();
    controller.h[4] = ESTIMATOR_GET_VEL_Y();
    controller.h[5] = MOTOR_VEL;

    controller.hd[0] = TRAJECTORY_GET_X(&trajectory);
    controller.hd[1] = TRAJECTORY_GET_Y(&trajectory);
    controller.hd[2] = MOTOR_VEL * controller.time;
    controller.hd[3] = TRAJECTORY_GET_D_X(&trajectory);
    controller.hd[4] = TRAJECTORY_GET_D_Y(&trajectory);
    controller.hd[5] = MOTOR_VEL;
    controller.hd[6] = TRAJECTORY_GET_D2_X(&trajectory);
    controller.hd[7] = TRAJECTORY_GET_D2_Y(&trajectory);
    controller.hd[8] = 0;

    float v[3];
    jptd_dynamic_1d_feedback_v(v, K1, K2, controller.h, controller.hd);

    controller.q[0] = ESTIMATOR_GET_POS_X();
    controller.q[1] = ESTIMATOR_GET_POS_Y();
    controller.q[2] = ESTIMATOR_GET_POS_THETA();
    controller.q[3] = phi1;
    controller.q[4] = phi2;
    controller.q[5] = MOTOR_VEL * controller.time;

    float eta[3] = {
        0,
        0,
        MOTOR_VEL,
    };

    float u[3];
    jptd_dynamic_1d_linearize_u(u, v, eta, controller.q);

    eta[0] = u[0];
    eta[1] = u[1];

    float input[2] = {0};
    input[INTEGRAL_IDX_PHI1] = eta[0];
    input[INTEGRAL_IDX_PHI2] = eta[1];
    integrator_step(&integrator, input);

    servos_set_position(integrator_get(&integrator, INTEGRAL_IDX_PHI1), 0,
                        integrator_get(&integrator, INTEGRAL_IDX_PHI2), 0);
}

STREAM_REGISTER("controller_continue", ok)
WATCHDOG_REGISTER(abort)
TASK_REGISTER_INIT(init)
TASK_REGISTER_PERIODIC(loop, 1000)
