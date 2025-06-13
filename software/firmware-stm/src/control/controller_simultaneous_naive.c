#include <math.h>
#include <string.h>

#include "actuate/motors.h"
#include "actuate/servos.h"
#include "com/stream.h"
#include "com/telemetry.h"
#include "control/robot_parameters.h"
#include "control/trajectory.h"
#include "control/watchdog.h"
#include "generated/estimator.h"
#include "utils/task.h"

#define CONTROLLER_K 2.f
#define MOTOR_VEL    -250.f
#define GIMBAL_MAX   (3.f * M_PI / 180.f)
#define CLAMP(val)                                                                                 \
    (((val) > (GIMBAL_MAX)) ? (GIMBAL_MAX) : (((val) < (-GIMBAL_MAX)) ? (-GIMBAL_MAX) : (val)))

// #define CONTROLS_IDENTICAL
#define CONTROLS_MIRRORED

typedef struct {
    bool started;
    uint32_t time_start;
    float time;
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

static void naive_control_law(const float *dhd, const float *hd, const float *q, float *ref) {
    const float cos_theta = cosf(q[2]);
    const float sin_theta = sinf(q[2]);

    const float x = dhd[0] - CONTROLLER_K * (q[0] - hd[0]);
    const float y = dhd[1] - CONTROLLER_K * (q[1] - hd[1]);

    ref[0] = +(x * cos_theta) + (y * sin_theta);
    ref[1] = -(x * sin_theta) + (y * cos_theta);
}

static float saturable_arcsin(const float x) {
    if(x > 1.f) {
        return M_PI / 2;
    }

    if(x < -1.f) {
        return -M_PI / 2;
    }

    return asinf(x);
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

    const float dhd[2] = {
        TRAJECTORY_GET_D_X(&trajectory),
        TRAJECTORY_GET_D_Y(&trajectory),
    };
    const float hd[2] = {
        TRAJECTORY_GET_X(&trajectory),
        TRAJECTORY_GET_Y(&trajectory),
    };
    const float q[3] = {
        estimator_state_get_x(),
        estimator_state_get_y(),
        estimator_state_get_theta(),
    };
    float ref[2];
    naive_control_law(dhd, hd, q, ref);

    const float theta_u = saturable_arcsin(-ref[1] / (ROBOT_PARAMETER_R * MOTOR_VEL));
    const float phi_u = saturable_arcsin(-ref[0] / (ROBOT_PARAMETER_R * MOTOR_VEL * cosf(theta_u)));

#if defined(CONTROLS_IDENTICAL)
    const float phi1 = phi_u;
    const float theta1 = theta_u;
    const float phi2 = phi_u;
    const float theta2 = theta_u;
#endif

#if defined(CONTROLS_MIRRORED)
    const float phi1 = phi_u;
    const float theta1 = theta_u;
    const float phi2 = -phi_u;
    const float theta2 = -theta_u;
#endif

    servos_set_position(CLAMP(phi1), CLAMP(theta1), CLAMP(phi2), CLAMP(theta2));
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
TASK_REGISTER_PERIODIC(loop, 10000)
TELEMETRY_REGISTER("controller", serialize, NULL)
