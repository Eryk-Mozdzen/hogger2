#include <string.h>

#include "com/stream.h"
#include "com/telemetry.h"
#include "control/generators.h"
#include "control/trajectory.h"
#include "utils/task.h"

#define PARAMS_NUM 10
#define READ_NUM   100

typedef void (*generator_t)(float *, const float *, const float);

static generator_t generator = NULL;
static float generator_params[PARAMS_NUM] = {0};
static float generator_output[TRAJECTORY_VALUES] = {0};

static bool read_started;
static uint32_t read_step;
static float read_time;

static void read_loop() {
    if(!read_started) {
        return;
    }

    static uint8_t buffer[2048];

    const float t = read_time * read_step / READ_NUM;

    float node[TRAJECTORY_VALUES] = {0};
    if(generator) {
        generator(node, generator_params, t);
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
                generator = generators_circle;
            } else if(strcmp(name, "lemniscate") == 0) {
                generator = generators_lemniscate;
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
        generator(generator_output, generator_params, t);
    }
    memcpy(trajectory->values, generator_output, sizeof(generator_output));
}

STREAM_REGISTER("trajectory_read", read_start)
STREAM_REGISTER("trajectory_write", write)
TASK_REGISTER_PERIODIC(read_loop, 10000)
TELEMETRY_REGISTER("trajectory", serialize, NULL)
