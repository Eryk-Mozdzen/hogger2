#include <cmp/cmp.h>
#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#include "utils/mpack.h"

static size_t writer(cmp_ctx_t *ctx, const void *data, size_t count) {
    mpack_t *mpack = (mpack_t *)ctx->buf;

    if((mpack->position + count) > mpack->capacity) {
        return 0;
    }

    memcpy(mpack->buffer + mpack->position, data, count);
    mpack->position += count;
    mpack->size = mpack->position;
    return count;
}

static bool reader(cmp_ctx_t *ctx, void *data, size_t count) {
    mpack_t *mpack = (mpack_t *)ctx->buf;

    if((mpack->position + count) > mpack->size) {
        return false;
    }

    memcpy(data, mpack->buffer + mpack->position, count);
    mpack->position += count;
    return true;
}

bool mpack_create_from(mpack_t *mpack, const uint8_t *buffer, const uint32_t size) {
    mpack->buffer = buffer;
    mpack->capacity = size;
    mpack->size = size;
    mpack->position = 0,

    cmp_init(&mpack->cmp, mpack, reader, NULL, writer);

    uint32_t map_size = 0;
    if(!cmp_read_map(&mpack->cmp, &map_size)) {
        return false;
    }
    if(map_size != 1) {
        return false;
    }

    return true;
}

void mpack_create_empty(mpack_t *mpack, uint8_t *buffer, const uint32_t capacity) {
    mpack->buffer = buffer;
    mpack->capacity = capacity;
    mpack->size = 0;
    mpack->position = 0;

    cmp_init(&mpack->cmp, mpack, reader, NULL, writer);

    cmp_write_map(&mpack->cmp, 1);
}

void mpack_copy(mpack_t *mpack, mpack_t *other) {
    *mpack = *other;
    mpack->cmp.buf = mpack;
}

bool mpack_read_bool(mpack_t *mpack, bool *value) {
    return cmp_read_bool(&mpack->cmp, value);
}

bool mpack_read_array(mpack_t *mpack, float *values, const uint32_t number) {
    uint32_t array_size = 0;
    if(!cmp_read_array(&mpack->cmp, &array_size)) {
        return false;
    }

    memset(values, 0, number * sizeof(float));

    for(uint8_t i = 0; i < array_size; i++) {
        float val = 0;
        if(!cmp_read_float(&mpack->cmp, &val)) {
            return false;
        }
        if(i < number) {
            values[i] = val;
        }
    }

    return true;
}
