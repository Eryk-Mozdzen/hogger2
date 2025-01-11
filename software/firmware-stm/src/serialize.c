#include <stdbool.h>
#include <stdint.h>
#include <string.h>
#include <cmp/cmp.h>

#include "serialize.h"
#include "robot.h"
#include "reference.h"

typedef struct {
    uint8_t *buffer;
    size_t capacity;
    size_t position;
    size_t size;
} buffer_t;

static const char *robot_state[] = {
    "stopped",
    "manual",
    "autonomous",
};

static const char *motor_state[] = {
    "idle",
    "startup",
    "startup",
    "startup",
    "running",
    "panic",
};

static size_t buffer_writer(cmp_ctx_t *ctx, const void *data, size_t count) {
    buffer_t *buf = (buffer_t *)ctx->buf;

    if((buf->position+count)>buf->capacity) {
        return 0;
    }

    memcpy(buf->buffer+buf->position, data, count);
    buf->position +=count;
    buf->size = buf->position;
    return count;
}

static bool buffer_reader(cmp_ctx_t *ctx, void *data, size_t count) {
    buffer_t *buf = (buffer_t *)ctx->buf;

    if((buf->position+count)>buf->size) {
        return false;
    }

    memcpy(data, buf->buffer + buf->position, count);
    buf->position +=count;
    return true;
}

uint32_t serialize_robot(const robot_t *robot, void *dest, const uint32_t dest_capacity) {
    buffer_t buffer = {
        .buffer = dest,
        .capacity = dest_capacity,
        .size = 0,
        .position = 0,
    };
    cmp_ctx_t cmp;
    cmp_init(&cmp, &buffer, NULL, NULL, buffer_writer);

    cmp_write_map(&cmp, 6);

    cmp_write_str(&cmp, "time", 4);
    cmp_write_u32(&cmp, robot->time);
    cmp_write_str(&cmp, "state", 5);
    cmp_write_str(&cmp, robot_state[robot->state], strlen(robot_state[robot->state]));
    cmp_write_str(&cmp, "supply", 6);
    cmp_write_float(&cmp, robot->supply);

    cmp_write_str(&cmp, "hog1", 4);
    cmp_write_map(&cmp, 4);
    cmp_write_str(&cmp, "servox", 6);
    cmp_write_float(&cmp, robot->hog[0]->servo_x.position);
    cmp_write_str(&cmp, "servoy", 6);
    cmp_write_float(&cmp, robot->hog[0]->servo_y.position);
    cmp_write_str(&cmp, "motor", 5);
    cmp_write_float(&cmp, robot->hog[0]->motor.vel);
    cmp_write_str(&cmp, "state", 5);
    cmp_write_str(&cmp, motor_state[robot->hog[0]->motor.state], strlen(motor_state[robot->hog[0]->motor.state]));

    cmp_write_str(&cmp, "hog2", 4);
    cmp_write_map(&cmp, 4);
    cmp_write_str(&cmp, "servox", 6);
    cmp_write_float(&cmp, robot->hog[1]->servo_x.position);
    cmp_write_str(&cmp, "servoy", 6);
    cmp_write_float(&cmp, robot->hog[1]->servo_y.position);
    cmp_write_str(&cmp, "motor", 5);
    cmp_write_float(&cmp, robot->hog[1]->motor.vel);
    cmp_write_str(&cmp, "state", 5);
    cmp_write_str(&cmp, motor_state[robot->hog[1]->motor.state], strlen(motor_state[robot->hog[1]->motor.state]));

    cmp_write_str(&cmp, "sensors", 7);
    cmp_write_array(&cmp, 3);
    cmp_write_map(&cmp, 3);
    cmp_write_str(&cmp, "name", 4);
    cmp_write_str(&cmp, "accelerometer", 13);
    cmp_write_str(&cmp, "unit", 4);
    cmp_write_str(&cmp, "m/s^2", 5);
    cmp_write_str(&cmp, "data", 4);
    cmp_write_array(&cmp, 3);
    cmp_write_float(&cmp, robot->imu->acceleration[0]);
    cmp_write_float(&cmp, robot->imu->acceleration[1]);
    cmp_write_float(&cmp, robot->imu->acceleration[2]);
    cmp_write_map(&cmp, 3);
    cmp_write_str(&cmp, "name", 4);
    cmp_write_str(&cmp, "gyroscope", 9);
    cmp_write_str(&cmp, "unit", 4);
    cmp_write_str(&cmp, "rad/s", 5);
    cmp_write_str(&cmp, "data", 4);
    cmp_write_array(&cmp, 3);
    cmp_write_float(&cmp, robot->imu->gyration[0]);
    cmp_write_float(&cmp, robot->imu->gyration[1]);
    cmp_write_float(&cmp, robot->imu->gyration[2]);
    cmp_write_map(&cmp, 3);
    cmp_write_str(&cmp, "name", 4);
    cmp_write_str(&cmp, "optical_flow", 12);
    cmp_write_str(&cmp, "unit", 4);
    cmp_write_str(&cmp, "1/s", 5);
    cmp_write_str(&cmp, "data", 4);
    cmp_write_array(&cmp, 2);
    cmp_write_float(&cmp, robot->flow->velocity[0]);
    cmp_write_float(&cmp, robot->flow->velocity[1]);

    return buffer.size;
}

bool deserialize_reference(const void *src, const uint32_t src_size, reference_t *reference) {
    buffer_t buffer = {
        .buffer = (void *)src,
        .capacity = src_size,
        .size = src_size,
        .position = 0,
    };
    cmp_ctx_t cmp;
    cmp_init(&cmp, &buffer, buffer_reader, NULL, NULL);

    uint32_t map_size = 0;
    if(!cmp_read_map(&cmp, &map_size)) {
        return false;
    }
    if(map_size!=2) {
        return false;
    }

    char key[32] = {0};
    uint32_t key_size = sizeof(key);
    if(!cmp_read_str(&cmp, key, &key_size)) {
        return false;
    }
    if(strncmp(key, "command", key_size)!=0) {
        return false;
    }

    char command[32] = {0};
    uint32_t command_size = sizeof(command);
    if(!cmp_read_str(&cmp, command, &command_size)) {
        return false;
    }
    if(strncmp(command, "manual", command_size)!=0) {
        return false;
    }

    key_size = sizeof(key);
    if(!cmp_read_str(&cmp, key, &key_size)) {
        return false;
    }
    if(strncmp(key, "ref_cfg", key_size)!=0) {
        return false;
    }

    uint32_t array_size = 0;
    if(!cmp_read_array(&cmp, &array_size)) {
        return false;
    }
    if(array_size!=6) {
        return false;
    }

    for(uint8_t i=0; i<6; i++) {
        float floating = 0;
        if(!cmp_read_float(&cmp, &floating)) {
            reference->configuration[i] = floating;
            return false;
        }
    }

    reference->type = REFERENCE_TYPE_CONFIGURATION;

    return true;
}
