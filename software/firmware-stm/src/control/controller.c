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
#define CONTROLLER_T        2.f
#define INTEGRAL_DIM        6
#define GENERATOR_PARAMS    10
#define TRAJECTORY_READ_NUM 100
#define MOTOR1_VEL          -300.f
#define MOTOR2_VEL          +300.f
#define GIMBAL_MAX          (7.f * DEG2RAD)

#define CLAMP(val, min, max) (((val) > (max)) ? (max) : (((val) < (min)) ? (min) : (val)))

typedef enum {
    INTEGRAL_IDX_PHI1,
    INTEGRAL_IDX_THETA1,
    INTEGRAL_IDX_PHI2,
    INTEGRAL_IDX_THETA2,
    INTEGRAL_IDX_ETA3,
    INTEGRAL_IDX_ETA5,
} integral_idx_t;

typedef void (*generator_t)(float *, const float);

typedef struct {
    generator_t generator;
    float generator_params[GENERATOR_PARAMS];

    bool read_started;
    uint32_t read_step;
    float read_time;
} trajectory_t;

typedef struct {
    float x0;
    float y0;
    float theta0;
    float heading;
    int k;
} smoother_t;

typedef struct {
    bool started;
    uint32_t time_prev;
    uint32_t time_start;
    float time;
    float h[10];
    float hd[12];
    float href[12];
} controller_t;

typedef struct {
    float prev[INTEGRAL_DIM];
    float integral[INTEGRAL_DIM];
    float input[INTEGRAL_DIM];
} integrator_t;

static trajectory_t trajectory = {0};
static smoother_t smoother = {0};
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

static void generator_circle(float *hd, const float t) {
    const float x = trajectory.generator_params[0];
    const float y = trajectory.generator_params[1];
    const float r = trajectory.generator_params[2];
    const float T = trajectory.generator_params[3];
    const float w = 2 * PI / T;

    hd[0] = x + r * cos(w * t);
    hd[1] = y + r * sin(w * t);
    hd[2] = w * t + PI / 2 + PI / 4; // very important pi/4 !!!

    hd[3] = -r * w * sin(w * t);
    hd[4] = r * w * cos(w * t);
    hd[5] = w;

    hd[6] = -r * w * w * cos(w * t);
    hd[7] = -r * w * w * sin(w * t);
    hd[8] = 0;

    hd[9] = r * w * w * w * sin(w * t);
    hd[10] = -r * w * w * w * cos(w * t);
    hd[11] = 0;
}

static void generator_lemniscate(float *hd, const float t) {
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

    hd[3] = a * w * (pow(sin(t * w), 2) - 3) * sin(t * w) / pow(pow(sin(t * w), 2) + 1, 2);
    hd[4] = a * w * (1 - 3 * pow(sin(t * w), 2)) / pow(pow(sin(t * w), 2) + 1, 2);
    hd[5] = 3 * w * cos(t * w) / (pow(sin(t * w), 2) + 1);

    hd[6] = a * pow(w, 2) * (-pow(sin(t * w), 4) + 12 * pow(sin(t * w), 2) - 3) * cos(t * w) /
            pow(pow(sin(t * w), 2) + 1, 3);
    hd[7] =
        2 * a * pow(w, 2) * (14 * sin(2 * t * w) + 3 * sin(4 * t * w)) / pow(cos(2 * t * w) - 3, 3);
    hd[8] = pow(w, 2) *
            (-19 * pow(sin(t * w), 10) + 31 * pow(sin(t * w), 8) + 17 * pow(sin(t * w), 6) -
             33 * pow(sin(t * w), 4) * pow(cos(t * w), 6) - 41 * pow(sin(t * w), 4) -
             10 * pow(sin(t * w), 2) + 14 * pow(cos(t * w), 10) - 2 * pow(cos(t * w), 8) -
             19 * pow(cos(t * w), 6) - 2) *
            sin(t * w) / pow(pow(sin(t * w), 2) + 1, 4);

    hd[9] = a * pow(w, 3) *
            (-pow(sin(t * w), 6) + 43 * pow(sin(t * w), 4) - 103 * pow(sin(t * w), 2) + 45) *
            sin(t * w) / pow(pow(sin(t * w), 2) + 1, 4);
    hd[10] = 2 * a * pow(w, 3) *
             (6 * pow(sin(t * w), 6) - 41 * pow(sin(t * w), 4) + 44 * pow(sin(t * w), 2) - 5) /
             pow(pow(sin(t * w), 2) + 1, 4);
    hd[11] = pow(w, 3) *
             (154 * pow(sin(t * w), 16) - 117 * pow(sin(t * w), 14) - 351 * pow(sin(t * w), 12) +
              370 * pow(sin(t * w), 10) * pow(cos(t * w), 6) + 228 * pow(sin(t * w), 10) +
              385 * pow(sin(t * w), 8) * pow(cos(t * w), 8) +
              404 * pow(sin(t * w), 8) * pow(cos(t * w), 6) + 276 * pow(sin(t * w), 8) +
              239 * pow(sin(t * w), 6) * pow(cos(t * w), 10) +
              197 * pow(sin(t * w), 6) * pow(cos(t * w), 8) -
              16 * pow(sin(t * w), 6) * pow(cos(t * w), 6) - 21 * pow(sin(t * w), 6) +
              82 * pow(sin(t * w), 4) * pow(cos(t * w), 12) +
              21 * pow(sin(t * w), 4) * pow(cos(t * w), 10) -
              75 * pow(sin(t * w), 4) * pow(cos(t * w), 8) -
              52 * pow(sin(t * w), 4) * pow(cos(t * w), 6) + 17 * pow(sin(t * w), 4) +
              6 * pow(sin(t * w), 2) - 12 * pow(cos(t * w), 16) + 24 * pow(cos(t * w), 14) +
              9 * pow(cos(t * w), 12) - 11 * pow(cos(t * w), 10) - 17 * pow(cos(t * w), 8) -
              2 * pow(cos(t * w), 6)) *
             cos(t * w) / pow(pow(sin(t * w), 2) + 1, 6);
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
    cmp_write_array(&response.cmp, 12);

    float node[12] = {0};
    if(trajectory.generator) {
        trajectory.generator(node, t);
    }
    for(uint32_t i = 0; i < 12; i++) {
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

        smoother.x0 = ESTIMATOR_GET_POS_X();
        smoother.y0 = ESTIMATOR_GET_POS_Y();
        smoother.theta0 = ESTIMATOR_GET_POS_THETA();
        smoother.heading = ESTIMATOR_GET_POS_THETA();
        smoother.k = 0;

        servos_set_position(+0.0472104532, -0.0471579290, -0.0472104532, +0.0471579290);
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
        trajectory.generator(controller.hd, controller.time);
    }

    float phi1;
    float theta1;
    float phi2;
    float theta2;
    servos_get_position(&phi1, &theta1, &phi2, &theta2);

    if(controller.time < CONTROLLER_T) {
        jptd_dynamic_smooth(controller.href, smoother.x0, smoother.y0, smoother.theta0,
                            controller.hd, CONTROLLER_T, controller.time);
    } else {
        memcpy(controller.href, controller.hd, sizeof(controller.href));
    }

    const float same = fabs(smoother.heading - (controller.href[2] + 2 * (smoother.k + 0) * PI));
    const float more = fabs(smoother.heading - (controller.href[2] + 2 * (smoother.k + 1) * PI));
    const float less = fabs(smoother.heading - (controller.href[2] + 2 * (smoother.k - 1) * PI));
    if(more < same && more < less) {
        smoother.k++;
    } else if(less < same && less < more) {
        smoother.k--;
    }
    controller.href[2] += 2 * smoother.k * PI;
    smoother.heading = controller.href[2];

    const float K1[25] = {
        CONTROLLER_K1, 0, 0, 0, 0, 0, CONTROLLER_K1, 0, 0, 0, 0, 0, CONTROLLER_K1, 0, 0, 0, 0, 0,
        CONTROLLER_K1, 0, 0, 0, 0, 0, CONTROLLER_K1,
    };

    const float K2[25] = {
        CONTROLLER_K2, 0, 0, 0, 0, 0, CONTROLLER_K2, 0, 0, 0, 0, 0, CONTROLLER_K2, 0, 0, 0, 0, 0,
        CONTROLLER_K2, 0, 0, 0, 0, 0, CONTROLLER_K2,
    };

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

    const float href[15] = {
        controller.href[0],
        controller.href[1],
        controller.href[2],
        MOTOR1_VEL * controller.time,
        MOTOR2_VEL * controller.time,
        controller.href[3],
        controller.href[4],
        controller.href[5],
        MOTOR1_VEL,
        MOTOR2_VEL,
    };

    float v[5];
    jptd_dynamic_feedback_v(v, K1, K2, controller.h, href);

    const float q[9] = {
        ESTIMATOR_GET_POS_X(),
        ESTIMATOR_GET_POS_Y(),
        ESTIMATOR_GET_POS_THETA(),
        phi1,
        theta1,
        MOTOR1_VEL * controller.time,
        phi2,
        theta2,
        MOTOR2_VEL * controller.time,
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

    servos_set_position(out_phi1, out_theta1, out_phi2, out_theta2);
}

static void trajectory_serialize(cmp_ctx_t *cmp, void *context) {
    (void)context;

    cmp_write_map(cmp, 2);
    cmp_write_str(cmp, "hd", 2);
    cmp_write_array(cmp, 12);
    for(uint32_t i = 0; i < 12; i++) {
        cmp_write_float(cmp, controller.hd[i]);
    }
    cmp_write_str(cmp, "href", 4);
    cmp_write_array(cmp, 12);
    for(uint32_t i = 0; i < 12; i++) {
        cmp_write_float(cmp, controller.href[i]);
    }
}

static void controller_serialize(cmp_ctx_t *cmp, void *context) {
    (void)context;

    cmp_write_map(cmp, 4);

    cmp_write_str(cmp, "started", 7);
    cmp_write_bool(cmp, controller.started);
    cmp_write_str(cmp, "time", 4);
    cmp_write_float(cmp, controller.time);
    cmp_write_str(cmp, "h", 1);
    cmp_write_array(cmp, 10);
    for(uint32_t i = 0; i < 10; i++) {
        cmp_write_float(cmp, controller.h[i]);
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
