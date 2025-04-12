#include "actuate/motors.h"
#include "actuate/servos.h"
#include "com/stream.h"
#include "com/telemetry.h"
#include "control/jptd_dynamic.h"
#include "control/watchdog.h"
#include "generated/estimator.h"
#include "utils/task.h"

#define DEG2RAD            (PI / 180.f)
#define RAD2DEG            (180.f / PI)
#define CONTROLLER_K1      2.f
#define CONTROLLER_K2      3.f
#define INTEGRAL_DIM       7
#define GENERATOR_PARAMS   10
#define TRAJECTORY_READ_DT 0.1f
#define TRAJECTORY_READ_T  10.f

typedef enum {
    INTEGRAL_IDX_PHI1,
    INTEGRAL_IDX_THETA1,
    INTEGRAL_IDX_THETA2,
    INTEGRAL_IDX_ETA3,
    INTEGRAL_IDX_ETA5,
    INTEGRAL_IDX_PSI1,
    INTEGRAL_IDX_PSI2,
} integral_idx_t;

typedef void (*generator_t)(float *, float *, float *, const float);

typedef struct {
    generator_t generator;
    float params[GENERATOR_PARAMS];

    bool read_started;
    float read_time;
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

static float angle_fix(float angle) {
    while(angle > (270 * DEG2RAD)) {
        angle -= 2 * PI;
    }

    while(angle < (-90 * DEG2RAD)) {
        angle += 2 * PI;
    }

    return angle;
}

static void generator_circle(float *hd, float *d_hd, float *d2_hd, const float t) {
    const float x = trajectory.params[0];
    const float y = trajectory.params[1];
    const float r = trajectory.params[2];
    const float T = trajectory.params[3];
    const float w = 2 * PI / T;

    hd[0] = x + r * cos(w * t);
    hd[1] = y + r * sin(w * t);
    hd[2] = w * t + PI / 2 - PI / 4; // very important -pi/4 !!!
    hd[3] = -300 * t;
    hd[4] = +300 * t;

    d_hd[0] = -r * w * sin(w * t);
    d_hd[1] = r * w * cos(w * t);
    d_hd[2] = w;
    d_hd[3] = -300;
    d_hd[4] = +300;

    d2_hd[0] = -r * w * w * cos(w * t);
    d2_hd[1] = -r * w * w * sin(w * t);
    d2_hd[2] = 0;
    d2_hd[3] = 0;
    d2_hd[4] = 0;
}

static void generator_lemniscate(float *hd, float *d_hd, float *d2_hd, const float t) {
    const float a = trajectory.params[0];
    const float T = trajectory.params[1];
    const float w = 2 * PI / T;

    hd[0] = a * cos(t * w) / (pow(sin(t * w), 2) + 1);
    hd[1] = a * sin(t * w) * cos(t * w) / (pow(sin(t * w), 2) + 1);
    hd[2] = angle_fix(
        atan2(-a * w * pow(sin(t * w), 2) / (pow(sin(t * w), 2) + 1) +
                  a * w * pow(cos(t * w), 2) / (pow(sin(t * w), 2) + 1) -
                  2 * a * w * pow(sin(t * w), 2) * pow(cos(t * w), 2) /
                      pow(pow(sin(t * w), 2) + 1, 2),
              -a * w * sin(t * w) / (pow(sin(t * w), 2) + 1) -
                  2 * a * w * sin(t * w) * pow(cos(t * w), 2) / pow(pow(sin(t * w), 2) + 1, 2)) -
        PI / 4); // very important -pi/4 !!!
    hd[3] = -300 * t;
    hd[4] = +300 * t;

    d_hd[0] = a * w * (pow(sin(t * w), 2) - 3) * sin(t * w) / pow(pow(sin(t * w), 2) + 1, 2);
    d_hd[1] = a * w * (1 - 3 * pow(sin(t * w), 2)) / pow(pow(sin(t * w), 2) + 1, 2);
    d_hd[2] = 3 * w * cos(t * w) / (pow(sin(t * w), 2) + 1);
    d_hd[3] = -300;
    d_hd[4] = +300;

    d2_hd[0] = a * pow(w, 2) * (-pow(sin(t * w), 4) + 12 * pow(sin(t * w), 2) - 3) * cos(t * w) /
               pow(pow(sin(t * w), 2) + 1, 3);
    d2_hd[1] =
        2 * a * pow(w, 2) * (14 * sin(2 * t * w) + 3 * sin(4 * t * w)) / pow(cos(2 * t * w) - 3, 3);
    d2_hd[2] = pow(w, 2) *
               (-19 * pow(sin(t * w), 10) + 31 * pow(sin(t * w), 8) + 17 * pow(sin(t * w), 6) -
                33 * pow(sin(t * w), 4) * pow(cos(t * w), 6) - 41 * pow(sin(t * w), 4) -
                10 * pow(sin(t * w), 2) + 14 * pow(cos(t * w), 10) - 2 * pow(cos(t * w), 8) -
                19 * pow(cos(t * w), 6) - 2) *
               sin(t * w) / pow(pow(sin(t * w), 2) + 1, 4);
    d2_hd[3] = 0;
    d2_hd[4] = 0;
}

static void trajectory_read_loop() {
    if(!trajectory.read_started) {
        return;
    }

    static uint8_t buffer[2048];

    mpack_t response;
    mpack_create_empty(&response, buffer, sizeof(buffer));

    cmp_write_str(&response.cmp, "trajectory", 10);
    cmp_write_map(&response.cmp, 2);
    cmp_write_str(&response.cmp, "time", 4);
    cmp_write_float(&response.cmp, trajectory.read_time);
    cmp_write_str(&response.cmp, "node", 4);
    cmp_write_array(&response.cmp, 15);

    float node[15] = {0};
    if(trajectory.generator) {
        trajectory.generator(&node[0], &node[5], &node[10], trajectory.read_time);
    }
    for(uint32_t i = 0; i < 15; i++) {
        cmp_write_float(&response.cmp, node[i]);
    }

    stream_transmit(&response);

    trajectory.read_time += TRAJECTORY_READ_DT;

    if(trajectory.read_time >= TRAJECTORY_READ_T) {
        trajectory.read_started = false;
    }
}

static void trajectory_read_start(mpack_t *mpack) {
    (void)mpack;

    trajectory.read_started = true;
    trajectory.read_time = 0;
}

static void trajectory_write(mpack_t *mpack) {
    uint32_t map_size = 0;
    if(!cmp_read_map(&mpack->cmp, &map_size)) {
        return;
    }

    for(uint32_t i = 0; i < map_size; i++) {
        char key[32] = {0};
        uint32_t key_size = sizeof(key);
        if(!cmp_read_str(&mpack->cmp, key, &key_size)) {
            return;
        }

        if(strncmp(key, "generator", key_size) == 0) {
            char name[32] = {0};
            if(!mpack_read_str(mpack, name, sizeof(name))) {
                return;
            }
            if(strcmp(name, "circle") == 0) {
                trajectory.generator = generator_circle;
            } else if(strcmp(name, "lemniscate") == 0) {
                trajectory.generator = generator_lemniscate;
            }
        } else if(strncmp(key, "params", key_size) == 0) {
            if(!mpack_read_float32_array(mpack, trajectory.params, GENERATOR_PARAMS)) {
                return;
            }
        }
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

    if(trajectory.generator) {
        trajectory.generator(controller.hd, controller.d_hd, controller.d2_hd, controller.time);
    }

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

    cmp_write_map(cmp, 3);
    cmp_write_str(cmp, "hd", 2);
    cmp_write_array(cmp, 5);
    for(uint32_t i = 0; i < 5; i++) {
        cmp_write_float(cmp, controller.hd[i]);
    }
    cmp_write_str(cmp, "d_hd", 4);
    cmp_write_array(cmp, 5);
    for(uint32_t i = 0; i < 5; i++) {
        cmp_write_float(cmp, controller.d_hd[i]);
    }
    cmp_write_str(cmp, "d2_hd", 5);
    cmp_write_array(cmp, 5);
    for(uint32_t i = 0; i < 5; i++) {
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
