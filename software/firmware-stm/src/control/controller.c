#include "actuate/motors.h"
#include "actuate/servos.h"
#include "com/stream.h"
#include "com/telemetry.h"
#include "control/jptd_dynamic.h"
#include "control/watchdog.h"
#include "generated/estimator.h"
#include "utils/task.h"

#define DEG2RAD             (PI / 180.f)
#define RAD2DEG             (180.f / PI)
#define CONTROLLER_K1       5.f
#define CONTROLLER_K2       10.f
#define INTEGRAL_DIM        6
#define GENERATOR_PARAMS    10
#define TRAJECTORY_READ_NUM 100
#define MOTOR1_VEL          -200.f
#define MOTOR2_VEL          +200.f
#define GIMBAL_MAX          (2.f * DEG2RAD)

#define CLAMP(val, min, max) (((val) > (max)) ? (max) : (((val) < (min)) ? (min) : (val)))

typedef enum {
    INTEGRAL_IDX_PHI1,
    INTEGRAL_IDX_THETA1,
    INTEGRAL_IDX_PHI2,
    INTEGRAL_IDX_THETA2,
    INTEGRAL_IDX_ETA3,
    INTEGRAL_IDX_ETA5,
} integral_idx_t;

typedef void (*generator_t)(float *, float *, float *, const float);

typedef struct {
    generator_t generator;
    float generator_params[GENERATOR_PARAMS];

    bool read_started;
    uint32_t read_step;
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
    const float x = trajectory.generator_params[0];
    const float y = trajectory.generator_params[1];
    const float r = trajectory.generator_params[2];
    const float T = trajectory.generator_params[3];
    const float w = 2 * PI / T;

    hd[0] = x + r * cos(w * t);
    hd[1] = y + r * sin(w * t);
    hd[2] = w * t + PI / 2 + PI / 4; // very important pi/4 !!!
    hd[3] = MOTOR1_VEL * t;
    hd[4] = MOTOR2_VEL * t;

    d_hd[0] = -r * w * sin(w * t);
    d_hd[1] = r * w * cos(w * t);
    d_hd[2] = w;
    d_hd[3] = MOTOR1_VEL;
    d_hd[4] = MOTOR2_VEL;

    d2_hd[0] = -r * w * w * cos(w * t);
    d2_hd[1] = -r * w * w * sin(w * t);
    d2_hd[2] = 0;
    d2_hd[3] = 0;
    d2_hd[4] = 0;
}

static void generator_lemniscate(float *hd, float *d_hd, float *d2_hd, const float t) {
    const float a = trajectory.generator_params[0];
    const float T = trajectory.generator_params[1];
    const float w = 2 * PI / T;

    hd[0] = a * cos(t * w) / (pow(sin(t * w), 2) + 1);
    hd[1] = a * sin(t * w) * cos(t * w) / (pow(sin(t * w), 2) + 1);
    hd[2] = angle_fix(
        atan2(-a * w * pow(sin(t * w), 2) / (pow(sin(t * w), 2) + 1) +
                  a * w * pow(cos(t * w), 2) / (pow(sin(t * w), 2) + 1) -
                  2 * a * w * pow(sin(t * w), 2) * pow(cos(t * w), 2) /
                      pow(pow(sin(t * w), 2) + 1, 2),
              -a * w * sin(t * w) / (pow(sin(t * w), 2) + 1) -
                  2 * a * w * sin(t * w) * pow(cos(t * w), 2) / pow(pow(sin(t * w), 2) + 1, 2)) +
        PI / 4); // very important pi/4 !!!
    hd[3] = MOTOR1_VEL * t;
    hd[4] = MOTOR2_VEL * t;

    d_hd[0] = a * w * (pow(sin(t * w), 2) - 3) * sin(t * w) / pow(pow(sin(t * w), 2) + 1, 2);
    d_hd[1] = a * w * (1 - 3 * pow(sin(t * w), 2)) / pow(pow(sin(t * w), 2) + 1, 2);
    d_hd[2] = 3 * w * cos(t * w) / (pow(sin(t * w), 2) + 1);
    d_hd[3] = MOTOR1_VEL;
    d_hd[4] = MOTOR2_VEL;

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

    const float t = trajectory.read_time * trajectory.read_step / TRAJECTORY_READ_NUM;

    mpack_t response;
    mpack_create_empty(&response, buffer, sizeof(buffer));

    cmp_write_str(&response.cmp, "trajectory", 10);
    cmp_write_map(&response.cmp, 2);
    cmp_write_str(&response.cmp, "time", 4);
    cmp_write_float(&response.cmp, t);
    cmp_write_str(&response.cmp, "node", 4);
    cmp_write_array(&response.cmp, 15);

    float node[15] = {0};
    if(trajectory.generator) {
        trajectory.generator(&node[0], &node[5], &node[10], t);
    }
    for(uint32_t i = 0; i < 15; i++) {
        cmp_write_float(&response.cmp, node[i]);
    }

    stream_transmit(&response);

    trajectory.read_step++;

    if(trajectory.read_step >= TRAJECTORY_READ_NUM) {
        trajectory.read_started = false;
    }
}

static void trajectory_read_start(mpack_t *mpack) {
    (void)mpack;

    if(!mpack_read_float32(mpack, &trajectory.read_time)) {
        return;
    }

    trajectory.read_started = true;
    trajectory.read_step = 0;
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
            if(!mpack_read_float32_array(mpack, trajectory.generator_params, GENERATOR_PARAMS)) {
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
        integrator.integral[INTEGRAL_IDX_ETA3] = MOTOR1_VEL;
        integrator.integral[INTEGRAL_IDX_ETA5] = MOTOR2_VEL;
    }
}

static void controller_abort() {
    controller.started = false;
}

static void controller_loop() {
    const uint32_t now = task_timebase();

    const float dt = (now - controller.time_prev) * 0.001f;
    controller.time = (now - controller.time_start) * 0.000001f;

    controller.time_prev = now;

    if(!controller.started) {
        memset(&controller, 0, sizeof(controller));
        memset(&integrator, 0, sizeof(integrator));
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

    float phi1;
    float theta1;
    float phi2;
    float theta2;
    servos_get_position(&phi1, &theta1, &phi2, &theta2);

    phi1 -= (+0.04015949507360143);
    phi2 -= (-0.058555490748248465),

        controller.h[0] = ESTIMATOR_GET_POS_X();
    controller.h[1] = ESTIMATOR_GET_POS_Y();
    controller.h[2] = ESTIMATOR_GET_POS_THETA();
    controller.h[3] = controller.hd[3];
    controller.h[4] = controller.hd[4];

    controller.d_h[0] = ESTIMATOR_GET_VEL_X();
    controller.d_h[1] = ESTIMATOR_GET_VEL_Y();
    controller.d_h[2] = ESTIMATOR_GET_VEL_THETA();
    controller.d_h[3] = controller.d_hd[3];
    controller.d_h[4] = controller.d_hd[4];

    float v[5];
    jptd_dynamic_feedback_v(v, K1, K2, controller.h, controller.d_h, controller.hd, controller.d_hd,
                            controller.d2_hd);

    const float q[9] = {
        ESTIMATOR_GET_POS_X(),
        ESTIMATOR_GET_POS_Y(),
        ESTIMATOR_GET_POS_THETA(),
        phi1,
        theta1,
        controller.hd[3],
        phi2 + 0.01f * expf(-100000.f * phi2 * phi2),       // for non-zero values of phi_2
        theta2 + 0.01f * expf(-100000.f * theta2 * theta2), // for non-zero values of theta_2
        controller.hd[4],
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
    integrator.input[INTEGRAL_IDX_PHI2] = eta[0] + sinf(theta1) * eta[2] - sinf(theta2) * eta[4];
    integrator.input[INTEGRAL_IDX_THETA2] = eta[3];
    integrator.input[INTEGRAL_IDX_ETA3] = u[2];
    integrator.input[INTEGRAL_IDX_ETA5] = u[4];

    float min[INTEGRAL_DIM] = {0};
    min[INTEGRAL_IDX_PHI1] = -GIMBAL_MAX;
    min[INTEGRAL_IDX_THETA1] = -GIMBAL_MAX;
    min[INTEGRAL_IDX_PHI2] = -GIMBAL_MAX;
    min[INTEGRAL_IDX_THETA2] = -GIMBAL_MAX;
    min[INTEGRAL_IDX_ETA3] = MOTOR1_VEL;
    min[INTEGRAL_IDX_ETA5] = MOTOR2_VEL;

    float max[INTEGRAL_DIM] = {0};
    max[INTEGRAL_IDX_PHI1] = GIMBAL_MAX;
    max[INTEGRAL_IDX_THETA1] = GIMBAL_MAX;
    max[INTEGRAL_IDX_PHI2] = GIMBAL_MAX;
    max[INTEGRAL_IDX_THETA2] = GIMBAL_MAX;
    max[INTEGRAL_IDX_ETA3] = MOTOR1_VEL;
    max[INTEGRAL_IDX_ETA5] = MOTOR2_VEL;

    for(uint32_t i = 0; i < INTEGRAL_DIM; i++) {
        integrator.integral[i] += 0.5f * (integrator.input[i] + integrator.prev[i]) * dt;
        integrator.integral[i] = CLAMP(integrator.integral[i], min[i], max[i]);
        integrator.prev[i] = integrator.input[i];
    }

    const float out_phi1 = integrator.integral[INTEGRAL_IDX_PHI1];
    const float out_theta1 = integrator.integral[INTEGRAL_IDX_THETA1];
    const float out_phi2 = integrator.integral[INTEGRAL_IDX_PHI2];
    const float out_theta2 = integrator.integral[INTEGRAL_IDX_THETA2];

    const float smooth = 0.5f * (tanhf(2.f * controller.time - 2.f) + 1.f);

    servos_set_position(smooth * out_phi1 + (+0.04015949507360143), smooth * out_theta1,
                        smooth * out_phi2 + (-0.058555490748248465), smooth * out_theta2);
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
TASK_REGISTER_PERIODIC(controller_loop, 1000)
TELEMETRY_REGISTER("trajectory", trajectory_serialize, NULL)
TELEMETRY_REGISTER("controller", controller_serialize, NULL)
