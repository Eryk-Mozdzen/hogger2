#include <math.h>
#include <string.h>

#include "actuate/motors.h"
#include "actuate/servos.h"
#include "com/stream.h"
#include "com/telemetry.h"
#include "control/integrator.h"
#include "control/jptd_dynamic_1d.h"
#include "control/robot_parameters.h"
#include "control/trajectory.h"
#include "control/watchdog.h"
#include "generated/estimator.h"
#include "utils/task.h"

#define CONTROLLER_K1 2.f
#define CONTROLLER_K2 4.f
#define MOTOR_VEL     -200.f
#define GIMBAL_MAX    (3.f * M_PI / 180.f)

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

    const float sin_theta = sinf(estimator_state_get_theta());
    const float cos_theta = cosf(estimator_state_get_theta());

    controller.h[0] = estimator_state_get_px() + ROBOT_PARAMETER_D * cos_theta;
    controller.h[1] = estimator_state_get_py() + ROBOT_PARAMETER_D * sin_theta;
    controller.h[2] = MOTOR_VEL * controller.time;
    controller.h[3] =
        estimator_state_get_vx() - ROBOT_PARAMETER_D * estimator_state_get_vtheta() * sin_theta;
    controller.h[4] =
        estimator_state_get_vy() + ROBOT_PARAMETER_D * estimator_state_get_vtheta() * cos_theta;
    controller.h[5] = MOTOR_VEL;

    trajectory_t trajectory;
    trajectory_get(&trajectory, controller.time);

    const float sin_theta_ref = sinf(TRAJECTORY_GET_THETA(&trajectory));
    const float cos_theta_ref = cosf(TRAJECTORY_GET_THETA(&trajectory));

    controller.hd[0] = TRAJECTORY_GET_X(&trajectory) + ROBOT_PARAMETER_D * cos_theta_ref;
    controller.hd[1] = TRAJECTORY_GET_Y(&trajectory) + ROBOT_PARAMETER_D * sin_theta_ref;
    controller.hd[2] = MOTOR_VEL * controller.time;
    controller.hd[3] = TRAJECTORY_GET_D_X(&trajectory) -
                       ROBOT_PARAMETER_D * TRAJECTORY_GET_D_THETA(&trajectory) * sin_theta_ref;
    controller.hd[4] = TRAJECTORY_GET_D_Y(&trajectory) +
                       ROBOT_PARAMETER_D * TRAJECTORY_GET_D_THETA(&trajectory) * cos_theta_ref;
    controller.hd[5] = MOTOR_VEL;
    controller.hd[6] =
        TRAJECTORY_GET_D2_X(&trajectory) -
        ROBOT_PARAMETER_D * (TRAJECTORY_GET_D2_THETA(&trajectory) * sin_theta_ref +
                             TRAJECTORY_GET_D_THETA(&trajectory) *
                                 TRAJECTORY_GET_D_THETA(&trajectory) * cos_theta_ref);
    controller.hd[7] =
        TRAJECTORY_GET_D2_Y(&trajectory) +
        ROBOT_PARAMETER_D * (TRAJECTORY_GET_D2_THETA(&trajectory) * cos_theta_ref -
                             TRAJECTORY_GET_D_THETA(&trajectory) *
                                 TRAJECTORY_GET_D_THETA(&trajectory) * sin_theta_ref);
    controller.hd[8] = 0;

    float v[3];
    jptd_dynamic_1d_feedback_v(v, K1, K2, controller.h, controller.hd);

    float phi1;
    float phi2;
    servos_get_position(&phi1, NULL, &phi2, NULL);

    controller.q[0] = estimator_state_get_px();
    controller.q[1] = estimator_state_get_py();
    controller.q[2] = estimator_state_get_theta();
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
