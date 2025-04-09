#include "actuate/motors.h"
#include "actuate/servos.h"
#include "com/stream.h"
#include "com/telemetry.h"
#include "control/jptd_dynamic.h"
#include "control/watchdog.h"
#include "generated/estimator.h"
#include "utils/task.h"

#define CONTROLLER_K1 2.f
#define CONTROLLER_K2 3.f

#define TRAJECTORY_NODES 1000
#define TRAJECTORY_DIM   5
#define TRAJECTORY_DERIV 3

#define INTEGRAL_DIM 7

typedef enum {
    INTEGRAL_IDX_PHI1,
    INTEGRAL_IDX_THETA1,
    INTEGRAL_IDX_THETA2,
    INTEGRAL_IDX_ETA3,
    INTEGRAL_IDX_ETA5,
    INTEGRAL_IDX_PSI1,
    INTEGRAL_IDX_PSI2,
} integral_idx_t;

typedef struct {
    float nodes[TRAJECTORY_NODES * TRAJECTORY_DIM * TRAJECTORY_DERIV];
    float dt;
    uint32_t total;

    bool read_started;
    uint32_t read_index;
} trajectory_t;

typedef struct {
    bool started;
    uint32_t time_prev;
    uint32_t time_start;
    float time;
    float hd[5];
    float d_hd[5];
    float d2_hd[5];
    float h[5];
    float d_h[5];
} controller_t;

typedef struct {
    float prev[INTEGRAL_DIM];
    float integral[INTEGRAL_DIM];
    float input[INTEGRAL_DIM];
} integrator_t;

static trajectory_t trajectory = {0};
static controller_t controller = {0};
static integrator_t integrator = {0};

static void trajectory_read_loop() {
    if(!trajectory.read_started) {
        return;
    }

    static uint8_t buffer[2048];

    mpack_t response;
    mpack_create_empty(&response, buffer, sizeof(buffer));

    cmp_write_str(&response.cmp, "trajectory", 10);
    cmp_write_map(&response.cmp, 2);
    cmp_write_str(&response.cmp, "index", 5);
    cmp_write_u32(&response.cmp, trajectory.read_index);
    cmp_write_str(&response.cmp, "node", 5);
    cmp_write_array(&response.cmp, TRAJECTORY_DIM * TRAJECTORY_DERIV);

    const float *node =
        &trajectory.nodes[trajectory.read_index * TRAJECTORY_DIM * TRAJECTORY_DERIV];

    for(uint32_t j = 0; j < (TRAJECTORY_DIM * TRAJECTORY_DERIV); j++) {
        cmp_write_float(&response.cmp, node[j]);
    }

    stream_transmit(&response);

    trajectory.read_index++;

    if(trajectory.read_index >= trajectory.total) {
        trajectory.read_started = false;
    }
}

static void trajectory_read_start(mpack_t *mpack) {
    (void)mpack;

    trajectory.read_started = true;
    trajectory.read_index = 0;
}

static void trajectory_write(mpack_t *mpack) {
    uint32_t map_size = 0;
    if(!cmp_read_map(&mpack->cmp, &map_size)) {
        return;
    }

    uint32_t start = 0;

    for(uint32_t i = 0; i < map_size; i++) {
        char key[32];
        uint32_t key_size = sizeof(key);
        if(!cmp_read_str(&mpack->cmp, key, &key_size)) {
            return;
        }

        if(strncmp(key, "total", key_size) == 0) {
            float val = 0;
            if(!cmp_read_float(&mpack->cmp, &val)) {
                return;
            }
            trajectory.total = val;
        } else if(strncmp(key, "dt", key_size) == 0) {
            if(!cmp_read_float(&mpack->cmp, &trajectory.dt)) {
                return;
            }
        } else if(strncmp(key, "start", key_size) == 0) {
            float val = 0;
            if(!cmp_read_float(&mpack->cmp, &val)) {
                return;
            }
            start = val;
        } else if(strncmp(key, "nodes", key_size) == 0) {
            uint32_t array_size = 0;
            if(!cmp_read_array(&mpack->cmp, &array_size)) {
                return;
            }

            for(uint8_t j = 0; j < array_size; j++) {
                float *node = &trajectory.nodes[(start + j) * TRAJECTORY_DIM * TRAJECTORY_DERIV];

                uint32_t inner_array_size = 0;
                if(!cmp_read_array(&mpack->cmp, &inner_array_size)) {
                    return;
                }

                if(inner_array_size != (TRAJECTORY_DIM * TRAJECTORY_DERIV)) {
                    return;
                }

                for(uint8_t k = 0; k < inner_array_size; k++) {
                    float val = 0;
                    if(!cmp_read_float(&mpack->cmp, &val)) {
                        return;
                    }

                    node[k] = val;
                }
            }
        }
    }
}

static void trajectory_interpolate(float *hd, float *d_hd, float *d2_hd, const float t) {
    // quintic Hermite interpolation

    const float T_total = (trajectory.total - 1) * trajectory.dt;
    const float t_mod = fmodf(t, T_total);

    uint32_t seg = (uint32_t)(t_mod / trajectory.dt);

    if(seg >= trajectory.total - 1) {
        seg = trajectory.total - 2;
    }

    const float t0 = seg * trajectory.dt;
    const float local_t = t_mod - t0;
    const float tau = local_t / trajectory.dt;

    const float tau2 = tau * tau;
    const float tau3 = tau2 * tau;
    const float tau4 = tau3 * tau;
    const float tau5 = tau4 * tau;

    const float h00 = 1.0f - 10.0f * tau3 + 15.0f * tau4 - 6.0f * tau5;
    const float h10 = tau - 6.0f * tau3 + 8.0f * tau4 - 3.0f * tau5;
    const float h20 = 0.5f * tau2 - 1.5f * tau3 + 1.5f * tau4 - 0.5f * tau5;
    const float h01 = 10.0f * tau3 - 15.0f * tau4 + 6.0f * tau5;
    const float h11 = -4.0f * tau3 + 7.0f * tau4 - 3.0f * tau5;
    const float h21 = 0.5f * tau3 - tau4 + 0.5f * tau5;

    const float dh00 = -30.0f * tau2 + 60.0f * tau3 - 30.0f * tau4;
    const float dh10 = 1.0f - 18.0f * tau2 + 32.0f * tau3 - 15.0f * tau4;
    const float dh20 = tau - 4.5f * tau2 + 6.0f * tau3 - 2.5f * tau4;
    const float dh01 = 30.0f * tau2 - 60.0f * tau3 + 30.0f * tau4;
    const float dh11 = -12.0f * tau2 + 28.0f * tau3 - 15.0f * tau4;
    const float dh21 = 1.5f * tau2 - 4.0f * tau3 + 2.5f * tau4;

    const float d2h00 = -60.0f * tau + 180.0f * tau2 - 120.0f * tau3;
    const float d2h10 = -36.0f * tau + 96.0f * tau2 - 60.0f * tau3;
    const float d2h20 = 1.0f - 9.0f * tau + 18.0f * tau2 - 10.0f * tau3;
    const float d2h01 = 60.0f * tau - 180.0f * tau2 + 120.0f * tau3;
    const float d2h11 = -24.0f * tau + 84.0f * tau2 - 60.0f * tau3;
    const float d2h21 = 3.0f * tau - 12.0f * tau2 + 10.0f * tau3;

    const float *p0 = &trajectory.nodes[seg * TRAJECTORY_DIM * TRAJECTORY_DERIV];
    const float *p1 = &trajectory.nodes[(seg + 1) * TRAJECTORY_DIM * TRAJECTORY_DERIV];

    const float T = trajectory.dt;
    const float T2 = T * T;

    for(uint32_t i = 0; i < TRAJECTORY_DIM; i++) {
        const float pos0 = p0[0 * TRAJECTORY_DIM + i];
        const float vel0 = p0[1 * TRAJECTORY_DIM + i];
        const float acc0 = p0[2 * TRAJECTORY_DIM + i];

        const float pos1 = p1[0 * TRAJECTORY_DIM + i];
        const float vel1 = p1[1 * TRAJECTORY_DIM + i];
        const float acc1 = p1[2 * TRAJECTORY_DIM + i];

        hd[i] = h00 * pos0 + (T * h10) * vel0 + (T2 * h20) * acc0 + h01 * pos1 + (T * h11) * vel1 +
                (T2 * h21) * acc1;

        const float dP_dtau = dh00 * pos0 + (T * dh10) * vel0 + (T2 * dh20) * acc0 + dh01 * pos1 +
                              (T * dh11) * vel1 + (T2 * dh21) * acc1;
        d_hd[i] = dP_dtau / T;

        const float d2P_dtau2 = d2h00 * pos0 + (T * d2h10) * vel0 + (T2 * d2h20) * acc0 +
                                d2h01 * pos1 + (T * d2h11) * vel1 + (T2 * d2h21) * acc1;
        d2_hd[i] = d2P_dtau2 / T2;
    }
}

static void controller_continue(mpack_t *mpack) {
    (void)mpack;

    watchdog_reset();

    if(!controller.started) {
        controller.started = true;
        controller.time_start = task_timebase();

        memset(integrator.prev, 0, sizeof(integrator.prev));
        memset(integrator.integral, 0, sizeof(integrator.integral));
        integrator.integral[INTEGRAL_IDX_ETA3] = -300;
        integrator.integral[INTEGRAL_IDX_ETA5] = +300;
    }
}

static void controller_abort() {
    controller.started = false;
}

static void controller_loop() {
    const uint32_t now = task_timebase();

    const float dt = (now - controller.time_prev) * 0.0001f;
    controller.time = (now - controller.time_start) * 0.000001f;

    controller.time_prev = now;

    if(!controller.started) {
        return;
    }

    trajectory_interpolate(controller.hd, controller.d_hd, controller.d2_hd, controller.time);

    const float K1[25] = {
        CONTROLLER_K1, 0, 0, 0, 0, 0, CONTROLLER_K1, 0, 0, 0, 0, 0, CONTROLLER_K1, 0, 0, 0, 0, 0,
        CONTROLLER_K1, 0, 0, 0, 0, 0, CONTROLLER_K1,
    };

    const float K2[25] = {
        CONTROLLER_K2, 0, 0, 0, 0, 0, CONTROLLER_K2, 0, 0, 0, 0, 0, CONTROLLER_K2, 0, 0, 0, 0, 0,
        CONTROLLER_K2, 0, 0, 0, 0, 0, CONTROLLER_K2,
    };

    float psi1_dot;
    float psi2_dot;
    motors_get_velocity(&psi1_dot, &psi2_dot);

    float phi1;
    float theta1;
    float phi2;
    float theta2;
    servos_get_position(&phi1, &theta1, &phi2, &theta2);

    controller.h[0] = ESTIMATOR_GET_POS_X();
    controller.h[1] = ESTIMATOR_GET_POS_Y();
    controller.h[2] = ESTIMATOR_GET_POS_THETA();
    controller.h[3] = integrator.integral[INTEGRAL_IDX_PHI1];
    controller.h[4] = integrator.integral[INTEGRAL_IDX_PSI2];

    controller.d_h[0] = ESTIMATOR_GET_VEL_X();
    controller.d_h[1] = ESTIMATOR_GET_VEL_Y();
    controller.d_h[2] = ESTIMATOR_GET_VEL_THETA();
    controller.d_h[3] = psi1_dot;
    controller.d_h[4] = psi2_dot;

    float v[5];
    jptd_dynamic_feedback_v(v, K1, K2, controller.h, controller.d_h, controller.hd, controller.d_hd,
                            controller.d2_hd);

    const float q[9] = {
        ESTIMATOR_GET_POS_X(),
        ESTIMATOR_GET_POS_Y(),
        ESTIMATOR_GET_POS_THETA(),
        phi1,
        theta1,
        integrator.integral[INTEGRAL_IDX_PSI1],
        phi2,
        theta2,
        integrator.integral[INTEGRAL_IDX_PSI2],
    };

    float eta[5] = {
        0, 0, integrator.integral[INTEGRAL_IDX_ETA3], 0, integrator.integral[INTEGRAL_IDX_ETA5],
    };

    float u[5];
    jptd_dynamic_linearize_u(u, v, eta, q);

    eta[0] = u[0];
    eta[1] = u[1];
    eta[3] = u[3];

    integrator.input[INTEGRAL_IDX_PHI1] = eta[0];
    integrator.input[INTEGRAL_IDX_THETA1] = eta[1];
    integrator.input[INTEGRAL_IDX_THETA2] = eta[3];
    integrator.input[INTEGRAL_IDX_ETA3] = u[2];
    integrator.input[INTEGRAL_IDX_ETA5] = u[4];
    integrator.input[INTEGRAL_IDX_PSI1] = psi1_dot;
    integrator.input[INTEGRAL_IDX_PSI2] = psi2_dot;

    for(uint32_t i = 0; i < INTEGRAL_DIM; i++) {
        integrator.integral[i] += 0.5f * (integrator.input[i] + integrator.prev[i]) * dt;
        integrator.prev[i] = integrator.integral[i];
    }

    const float setpoint_phi2 = integrator.integral[INTEGRAL_IDX_PHI1] +
                                sinf(integrator.integral[INTEGRAL_IDX_THETA1]) -
                                sinf(integrator.integral[INTEGRAL_IDX_THETA2]);

    motors_set_velocity(integrator.integral[INTEGRAL_IDX_ETA3],
                        integrator.integral[INTEGRAL_IDX_ETA5]);
    servos_set_position(integrator.integral[INTEGRAL_IDX_PHI1],
                        integrator.integral[INTEGRAL_IDX_THETA1], setpoint_phi2,
                        integrator.integral[INTEGRAL_IDX_THETA2]);
}

static void trajectory_serialize(cmp_ctx_t *cmp, void *context) {
    (void)context;

    cmp_write_map(cmp, 5);
    cmp_write_str(cmp, "dt", 2);
    cmp_write_float(cmp, trajectory.dt);
    cmp_write_str(cmp, "total", 5);
    cmp_write_u32(cmp, trajectory.total);
    cmp_write_str(cmp, "hd", 2);
    cmp_write_array(cmp, TRAJECTORY_DIM);
    for(uint32_t i = 0; i < TRAJECTORY_DIM; i++) {
        cmp_write_float(cmp, controller.hd[i]);
    }
    cmp_write_str(cmp, "d_hd", 4);
    cmp_write_array(cmp, TRAJECTORY_DIM);
    for(uint32_t i = 0; i < TRAJECTORY_DIM; i++) {
        cmp_write_float(cmp, controller.d_hd[i]);
    }
    cmp_write_str(cmp, "d2_hd", 5);
    cmp_write_array(cmp, TRAJECTORY_DIM);
    for(uint32_t i = 0; i < TRAJECTORY_DIM; i++) {
        cmp_write_float(cmp, controller.d2_hd[i]);
    }
}

static void controller_serialize(cmp_ctx_t *cmp, void *context) {
    (void)context;

    cmp_write_map(cmp, 5);

    cmp_write_str(cmp, "started", 7);
    cmp_write_bool(cmp, controller.started);
    cmp_write_str(cmp, "time", 4);
    cmp_write_float(cmp, controller.time);
    cmp_write_str(cmp, "h", 1);
    cmp_write_array(cmp, 5);
    for(uint32_t i = 0; i < 5; i++) {
        cmp_write_float(cmp, controller.h[i]);
    }
    cmp_write_str(cmp, "d_h", 3);
    cmp_write_array(cmp, 5);
    for(uint32_t i = 0; i < 5; i++) {
        cmp_write_float(cmp, controller.d_h[i]);
    }

    cmp_write_str(cmp, "integrator", 10);
    cmp_write_map(cmp, 2);
    cmp_write_str(cmp, "integral", 8);
    cmp_write_array(cmp, INTEGRAL_DIM);
    for(uint32_t i = 0; i < INTEGRAL_DIM; i++) {
        cmp_write_float(cmp, integrator.integral[i]);
    }
    cmp_write_str(cmp, "input", 5);
    cmp_write_array(cmp, INTEGRAL_DIM);
    for(uint32_t i = 0; i < INTEGRAL_DIM; i++) {
        cmp_write_float(cmp, integrator.input[i]);
    }
}

STREAM_REGISTER("trajectory_read", trajectory_read_start)
STREAM_REGISTER("trajectory_write", trajectory_write)
STREAM_REGISTER("controller_continue", controller_continue)
WATCHDOG_REGISTER(controller_abort)
TASK_REGISTER_PERIODIC(trajectory_read_loop, 10000)
TASK_REGISTER_PERIODIC(controller_loop, 100)
TELEMETRY_REGISTER("trajectory", trajectory_serialize, NULL)
TELEMETRY_REGISTER("controller", controller_serialize, NULL)
