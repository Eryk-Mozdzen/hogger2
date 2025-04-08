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

static float trajectory[TRAJECTORY_NODES * TRAJECTORY_DIM * TRAJECTORY_DERIV] = {0};
static float trajectory_dt = 0;
static uint32_t trajectory_total = 0;

static bool started = false;
static uint32_t start_time = 0;
static float controller_time = 0;
static float controller_hd[5] = {0};
static float controller_d_hd[5] = {0};
static float controller_d2_hd[5] = {0};

static float integral_value_prev[INTEGRAL_DIM] = {0};
static float integral[INTEGRAL_DIM] = {0};

static bool read_started = false;
static uint32_t read_index = 0;

static void read() {
    if(!read_started) {
        return;
    }

    static uint8_t buffer[2048];

    mpack_t response;
    mpack_create_empty(&response, buffer, sizeof(buffer));

    cmp_write_str(&response.cmp, "trajectory", 10);
    cmp_write_map(&response.cmp, 2);
    cmp_write_str(&response.cmp, "index", 5);
    cmp_write_u32(&response.cmp, read_index);
    cmp_write_str(&response.cmp, "node", 5);
    cmp_write_array(&response.cmp, TRAJECTORY_DIM * TRAJECTORY_DERIV);

    const float *node = &trajectory[read_index * TRAJECTORY_DIM * TRAJECTORY_DERIV];

    for(uint32_t j = 0; j < (TRAJECTORY_DIM * TRAJECTORY_DERIV); j++) {
        cmp_write_float(&response.cmp, node[j]);
    }

    stream_transmit(&response);

    read_index++;

    if(read_index >= trajectory_total) {
        read_started = false;
    }
}

static void trajectory_read(mpack_t *mpack) {
    (void)mpack;

    read_started = true;
    read_index = 0;
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
            trajectory_total = val;
        } else if(strncmp(key, "dt", key_size) == 0) {
            if(!cmp_read_float(&mpack->cmp, &trajectory_dt)) {
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
                float *node = &trajectory[(start + j) * TRAJECTORY_DIM * TRAJECTORY_DERIV];

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

static void trajectory_continue(mpack_t *mpack) {
    (void)mpack;

    watchdog_reset();

    if(!started) {
        started = true;
        start_time = task_timebase();

        memset(integral_value_prev, 0, sizeof(integral_value_prev));
        memset(integral, 0, sizeof(integral));
        integral[INTEGRAL_IDX_ETA3] = -300;
        integral[INTEGRAL_IDX_ETA5] = +300;
    }
}

static void trajectory_abort() {
    started = false;
}

static void trajectory_interpolate(float *hd, float *d_hd, float *d2_hd, const float t) {
    // quintic Hermite interpolation

    const float T_total = (trajectory_total - 1) * trajectory_dt;
    const float t_mod = fmodf(t, T_total);

    uint32_t seg = (uint32_t)(t_mod / trajectory_dt);

    if(seg >= trajectory_total - 1) {
        seg = trajectory_total - 2;
    }

    const float t0 = seg * trajectory_dt;
    const float local_t = t_mod - t0;
    const float tau = local_t / trajectory_dt;

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

    const float *p0 = &trajectory[seg * TRAJECTORY_DIM * TRAJECTORY_DERIV];
    const float *p1 = &trajectory[(seg + 1) * TRAJECTORY_DIM * TRAJECTORY_DERIV];

    const float T = trajectory_dt;
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

static void loop() {
    if(!started) {
        return;
    }

    static uint32_t prev = 0;
    const uint32_t now = task_timebase();

    const float dt = (now - prev) * 0.0001f;
    controller_time = (now - start_time) * 0.000001f;

    prev = now;

    trajectory_interpolate(controller_hd, controller_d_hd, controller_d2_hd, controller_time);

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

    const float h[5] = {
        ESTIMATOR_GET_POS_X(),       ESTIMATOR_GET_POS_Y(),       ESTIMATOR_GET_POS_THETA(),
        integral[INTEGRAL_IDX_PHI1], integral[INTEGRAL_IDX_PSI2],
    };

    const float d_h[5] = {
        ESTIMATOR_GET_VEL_X(), ESTIMATOR_GET_VEL_Y(), ESTIMATOR_GET_VEL_THETA(), psi1_dot, psi2_dot,
    };

    float v[5];
    jptd_dynamic_feedback_v(v, K1, K2, h, d_h, controller_hd, controller_d_hd, controller_d2_hd);

    const float q[9] = {
        ESTIMATOR_GET_POS_X(),
        ESTIMATOR_GET_POS_Y(),
        ESTIMATOR_GET_POS_THETA(),
        phi1,
        theta1,
        integral[INTEGRAL_IDX_PSI1],
        phi2,
        theta2,
        integral[INTEGRAL_IDX_PSI2],
    };

    float eta[5] = {
        0, 0, integral[INTEGRAL_IDX_ETA3], 0, integral[INTEGRAL_IDX_ETA5],
    };

    float u[5];
    jptd_dynamic_linearize_u(u, v, eta, q);

    eta[0] = u[0];
    eta[1] = u[1];
    eta[3] = u[3];

    float value[INTEGRAL_DIM] = {0};
    value[INTEGRAL_IDX_PHI1] = eta[0];
    value[INTEGRAL_IDX_THETA1] = eta[1];
    value[INTEGRAL_IDX_THETA2] = eta[3];
    value[INTEGRAL_IDX_ETA3] = u[2];
    value[INTEGRAL_IDX_ETA5] = u[4];
    value[INTEGRAL_IDX_PSI1] = psi1_dot;
    value[INTEGRAL_IDX_PSI2] = psi2_dot;

    for(uint32_t i = 0; i < INTEGRAL_DIM; i++) {
        integral[i] += 0.5f * (value[i] + integral_value_prev[i]) * dt;
        integral_value_prev[i] = integral[i];
    }

    const float setpoint_phi2 = integral[INTEGRAL_IDX_PHI1] + sinf(integral[INTEGRAL_IDX_THETA1]) -
                                sinf(integral[INTEGRAL_IDX_THETA2]);

    // motors_set_velocity(integral[INTEGRAL_IDX_ETA3], integral[INTEGRAL_IDX_ETA5]);
    servos_set_position(integral[INTEGRAL_IDX_PHI1], integral[INTEGRAL_IDX_THETA1], setpoint_phi2,
                        integral[INTEGRAL_IDX_THETA2]);
}

static void serialize(cmp_ctx_t *cmp, void *context) {
    (void)context;

    cmp_write_map(cmp, 4);
    cmp_write_str(cmp, "started", 7);
    cmp_write_bool(cmp, started);
    cmp_write_str(cmp, "time", 4);
    cmp_write_float(cmp, controller_time);
    cmp_write_str(cmp, "trajectory", 10);
    cmp_write_map(cmp, 5);
    cmp_write_str(cmp, "nodes_dt", 8);
    cmp_write_float(cmp, trajectory_dt);
    cmp_write_str(cmp, "nodes_total", 11);
    cmp_write_u32(cmp, trajectory_total);
    cmp_write_str(cmp, "pos", 3);
    cmp_write_array(cmp, TRAJECTORY_DIM);
    for(uint32_t i = 0; i < TRAJECTORY_DIM; i++) {
        cmp_write_float(cmp, controller_hd[i]);
    }
    cmp_write_str(cmp, "vel", 3);
    cmp_write_array(cmp, TRAJECTORY_DIM);
    for(uint32_t i = 0; i < TRAJECTORY_DIM; i++) {
        cmp_write_float(cmp, controller_d_hd[i]);
    }
    cmp_write_str(cmp, "acc", 3);
    cmp_write_array(cmp, TRAJECTORY_DIM);
    for(uint32_t i = 0; i < TRAJECTORY_DIM; i++) {
        cmp_write_float(cmp, controller_d2_hd[i]);
    }
    cmp_write_str(cmp, "integral", 8);
    cmp_write_array(cmp, INTEGRAL_DIM);
    for(uint32_t i = 0; i < INTEGRAL_DIM; i++) {
        cmp_write_float(cmp, integral[i]);
    }
}

STREAM_REGISTER("trajectory_read", trajectory_read)
STREAM_REGISTER("trajectory_write", trajectory_write)
STREAM_REGISTER("trajectory_continue", trajectory_continue)
WATCHDOG_REGISTER(trajectory_abort)
TASK_REGISTER_PERIODIC(read, 10000)
TASK_REGISTER_PERIODIC(loop, 100)
TELEMETRY_REGISTER("controller", serialize, NULL)
