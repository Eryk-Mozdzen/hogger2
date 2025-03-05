#ifndef STREAM_H
#define STREAM_H

#include <stdint.h>

#include "utils/mpack.h"

#define STREAM_CONCAT_IMPL(a, b) a##b
#define STREAM_CONCAT(a, b)      STREAM_CONCAT_IMPL(a, b)

#define STREAM_REGISTER(type, receiver)                                                            \
    __attribute__((constructor)) static void STREAM_CONCAT(stream_reg_, __LINE__)() {              \
        stream_register((type), (receiver));                                                       \
    }

typedef void (*stream_receiver_t)(mpack_t *);

void stream_register(const char *type, const stream_receiver_t receiver);
void stream_transmit(const char *type, const mpack_t *content);

#endif
