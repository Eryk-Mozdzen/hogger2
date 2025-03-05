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

size_t mpack_writer(cmp_ctx_t *ctx, const void *data, size_t count);
bool mpack_reader(cmp_ctx_t *ctx, void *data, size_t count);

bool mpack_read_map(mpack_t *mpack, const uint32_t size);
bool mpack_read_str(mpack_t *mpack, char *str);
bool mpack_read_array(mpack_t *mpack, float *values, const uint32_t number);

#endif
