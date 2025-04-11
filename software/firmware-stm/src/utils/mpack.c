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
    mpack->buffer = (uint8_t *)buffer;
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

bool mpack_read_uint32(mpack_t *mpack, uint32_t *value) {
    cmp_object_t object;
    if(!cmp_read_object(&mpack->cmp, &object)) {
        return false;
    }

    int8_t int8;
    if(cmp_object_as_char(&object, &int8)) {
        *value = int8;
        return true;
    }

    int16_t int16;
    if(cmp_object_as_short(&object, &int16)) {
        *value = int16;
        return true;
    }

    int32_t int32;
    if(cmp_object_as_int(&object, &int32)) {
        *value = int32;
        return true;
    }

    int64_t int64;
    if(cmp_object_as_long(&object, &int64)) {
        *value = int64;
        return true;
    }

    if(cmp_object_as_sinteger(&object, &int64)) {
        *value = int64;
        return true;
    }

    uint8_t uint8;
    if(cmp_object_as_uchar(&object, &uint8)) {
        *value = uint8;
        return true;
    }

    uint16_t uint16;
    if(cmp_object_as_ushort(&object, &uint16)) {
        *value = uint16;
        return true;
    }

    uint32_t uint32;
    if(cmp_object_as_uint(&object, &uint32)) {
        *value = uint32;
        return true;
    }

    uint64_t uint64;
    if(cmp_object_as_ulong(&object, &uint64)) {
        *value = uint64;
        return true;
    }

    if(cmp_object_as_uinteger(&object, &uint64)) {
        *value = uint64;
        return true;
    }

    float float32;
    if(cmp_object_as_float(&object, &float32)) {
        *value = float32;
        return true;
    }

    double float64;
    if(cmp_object_as_double(&object, &float64)) {
        *value = float64;
        return true;
    }

    return false;
}

bool mpack_read_float32(mpack_t *mpack, float *value) {
    cmp_object_t object;
    if(!cmp_read_object(&mpack->cmp, &object)) {
        return false;
    }

    int8_t int8;
    if(cmp_object_as_char(&object, &int8)) {
        *value = int8;
        return true;
    }

    int16_t int16;
    if(cmp_object_as_short(&object, &int16)) {
        *value = int16;
        return true;
    }

    int32_t int32;
    if(cmp_object_as_int(&object, &int32)) {
        *value = int32;
        return true;
    }

    int64_t int64;
    if(cmp_object_as_long(&object, &int64)) {
        *value = int64;
        return true;
    }

    if(cmp_object_as_sinteger(&object, &int64)) {
        *value = int64;
        return true;
    }

    uint8_t uint8;
    if(cmp_object_as_uchar(&object, &uint8)) {
        *value = uint8;
        return true;
    }

    uint16_t uint16;
    if(cmp_object_as_ushort(&object, &uint16)) {
        *value = uint16;
        return true;
    }

    uint32_t uint32;
    if(cmp_object_as_uint(&object, &uint32)) {
        *value = uint32;
        return true;
    }

    uint64_t uint64;
    if(cmp_object_as_ulong(&object, &uint64)) {
        *value = uint64;
        return true;
    }

    if(cmp_object_as_uinteger(&object, &uint64)) {
        *value = uint64;
        return true;
    }

    float float32;
    if(cmp_object_as_float(&object, &float32)) {
        *value = float32;
        return true;
    }

    double float64;
    if(cmp_object_as_double(&object, &float64)) {
        *value = float64;
        return true;
    }

    return false;
}

bool mpack_read_float32_array(mpack_t *mpack, float *values, const uint32_t size) {
    uint32_t array_size = 0;
    if(!cmp_read_array(&mpack->cmp, &array_size)) {
        return false;
    }

    memset(values, 0, size * sizeof(float));

    for(uint8_t i = 0; i < array_size; i++) {
        float val = 0;
        if(!mpack_read_float32(mpack, &val)) {
            return false;
        }
        if(i < size) {
            values[i] = val;
        }
    }

    return true;
}
