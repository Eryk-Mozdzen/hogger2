#ifndef MPACK_H
#define MPACK_H

#include <cmp/cmp.h>
#include <stdbool.h>
#include <stdint.h>

typedef struct {
    cmp_ctx_t cmp;
    uint8_t *buffer;
    size_t capacity;
    size_t position;
    size_t size;
} mpack_t;

bool mpack_create_from(mpack_t *mpack, const uint8_t *buffer, const uint32_t size);
void mpack_create_empty(mpack_t *mpack, uint8_t *buffer, const uint32_t capacity);
void mpack_copy(mpack_t *mpack, mpack_t *other);

bool mpack_read_bool(mpack_t *mpack, bool *value);
bool mpack_read_str(mpack_t *mpack, char *value, const uint32_t size);
bool mpack_read_uint32(mpack_t *mpack, uint32_t *value);
bool mpack_read_float32(mpack_t *mpack, float *value);
bool mpack_read_float32_array(mpack_t *mpack, float *values, const uint32_t size);

#endif
