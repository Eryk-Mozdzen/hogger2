#include <math.h>
#include <string.h>

#include "com/stream.h"
#include "com/telemetry.h"
#include "control/trajectory.h"
#include "utils/task.h"

#define PARAMS_NUM 10
#define READ_NUM   100

typedef void (*generator_t)(float *, const float);

static generator_t generator = NULL;
static float generator_params[PARAMS_NUM] = {0};
static float generator_output[TRAJECTORY_VALUES] = {0};

static bool read_started;
static uint32_t read_step;
static float read_time;

static float angle_fix(float angle) {
    while(angle > (270 * M_PI / 180.f)) {
        angle -= 2 * M_PI;
    }

    while(angle < (-90 * M_PI / 180.f)) {
        angle += 2 * M_PI;
    }

    return angle;
}

static void generator_circle(float *hd, const float t) {
    const float x = generator_params[0];
    const float y = generator_params[1];
    const float r = generator_params[2];
    const float T = generator_params[3];
    const float w = 2 * M_PI / T;

    hd[0] = x + r * cos(w * t);
    hd[1] = y + r * sin(w * t);
    hd[2] = w * t + M_PI / 2 + M_PI / 4; // very important pi/4 !!!

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
    const float a = generator_params[0];
    const float T = generator_params[1];
    const float w = 2 * M_PI / T;

    hd[0] = a * cos(t * w) / (pow(sin(t * w), 2) + 1);
    hd[1] = a * sin(t * w) * cos(t * w) / (pow(sin(t * w), 2) + 1);
    hd[2] = angle_fix(
        atan2(-a * w * pow(sin(t * w), 2) / (pow(sin(t * w), 2) + 1) +
                  a * w * pow(cos(t * w), 2) / (pow(sin(t * w), 2) + 1) -
                  2 * a * w * pow(sin(t * w), 2) * pow(cos(t * w), 2) /
                      pow(pow(sin(t * w), 2) + 1, 2),
              -a * w * sin(t * w) / (pow(sin(t * w), 2) + 1) -
                  2 * a * w * sin(t * w) * pow(cos(t * w), 2) / pow(pow(sin(t * w), 2) + 1, 2)) +
        M_PI / 4); // very important pi/4 !!!

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

static void read_loop() {
    if(!read_started) {
        return;
    }

    static uint8_t buffer[2048];

    const float t = read_time * read_step / READ_NUM;

    float node[TRAJECTORY_VALUES] = {0};
    if(generator) {
        generator(node, t);
    }

    mpack_t response;
    mpack_create_empty(&response, buffer, sizeof(buffer));

    cmp_write_str(&response.cmp, "trajectory", 10);
    cmp_write_map(&response.cmp, 2);
    cmp_write_str(&response.cmp, "time", 4);
    cmp_write_float(&response.cmp, t);
    cmp_write_str(&response.cmp, "node", 4);
    cmp_write_array(&response.cmp, TRAJECTORY_VALUES);
    for(uint32_t i = 0; i < TRAJECTORY_VALUES; i++) {
        cmp_write_float(&response.cmp, node[i]);
    }

    stream_transmit(&response);

    read_step++;

    if(read_step >= READ_NUM) {
        read_started = false;
    }
}

static void read_start(mpack_t *mpack) {
    (void)mpack;

    if(!mpack_read_float32(mpack, &read_time)) {
        return;
    }

    read_started = true;
    read_step = 0;
}

static void write(mpack_t *mpack) {
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
                generator = generator_circle;
            } else if(strcmp(name, "lemniscate") == 0) {
                generator = generator_lemniscate;
            }
        } else if(strncmp(key, "params", key_size) == 0) {
            if(!mpack_read_float32_array(mpack, generator_params, PARAMS_NUM)) {
                return;
            }
        }
    }
}

static void serialize(cmp_ctx_t *cmp, void *context) {
    (void)context;

    cmp_write_array(cmp, TRAJECTORY_VALUES);
    for(uint32_t i = 0; i < TRAJECTORY_VALUES; i++) {
        cmp_write_float(cmp, generator_output[i]);
    }
}

void trajectory_get(trajectory_t *trajectory, const float t) {
    memset(generator_output, 0, sizeof(generator_output));
    if(generator) {
        generator(generator_output, t);
    }
    memcpy(trajectory->values, generator_output, sizeof(generator_output));
}

STREAM_REGISTER("trajectory_read", read_start)
STREAM_REGISTER("trajectory_write", write)
TASK_REGISTER_PERIODIC(read_loop, 10000)
TELEMETRY_REGISTER("trajectory", serialize, NULL)
