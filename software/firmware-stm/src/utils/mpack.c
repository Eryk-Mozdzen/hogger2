#include <cmp/cmp.h>
#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#include "utils/mpack.h"

size_t mpack_writer(cmp_ctx_t *ctx, const void *data, size_t count) {
    mpack_t *mpack = (mpack_t *) ctx->buf;

    if((mpack->position + count) > mpack->capacity) {
        return 0;
    }

    memcpy(mpack->buffer + mpack->position, data, count);
    mpack->position += count;
    mpack->size = mpack->position;
    return count;
}

bool mpack_reader(cmp_ctx_t *ctx, void *data, size_t count) {
    mpack_t *mpack = (mpack_t *) ctx->buf;

    if((mpack->position + count) > mpack->size) {
        return false;
    }

    memcpy(data, mpack->buffer + mpack->position, count);
    mpack->position += count;
    return true;
}

bool mpack_read_map(mpack_t *mpack, const uint32_t size) {
    uint32_t map_size = 0;
    if(!cmp_read_map(&mpack->cmp, &map_size)) {
        return false;
    }
    if(map_size != size) {
        return false;
    }
    return true;
}

bool mpack_read_str(mpack_t *mpack, char *str) {
    char buffer[32] = {0};
    uint32_t buffer_size = sizeof(buffer);
    if(!cmp_read_str(&mpack->cmp, buffer, &buffer_size)) {
        return false;
    }
    const bool match = (strncmp(buffer, str, buffer_size) == 0);
    strncpy(str, buffer, buffer_size);
    return match;
}

bool mpack_read_array(mpack_t *mpack, float *values, const uint32_t number) {
    uint32_t array_size = 0;
    if(!cmp_read_array(&mpack->cmp, &array_size)) {
        return false;
    }
    if(array_size != number) {
        return false;
    }

    for(uint8_t i = 0; i < number; i++) {
        if(!cmp_read_float(&mpack->cmp, &values[i])) {
            return false;
        }
    }

    return true;
}
