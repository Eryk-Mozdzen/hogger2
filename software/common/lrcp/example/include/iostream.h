#ifndef IOSTREAM_H
#define IOSTREAM_H

#include <lrcp/stream.h>

typedef struct {
    lrcp_stream_t base;
    uint8_t buffer[10*1024];
    uint32_t rx;
    uint32_t tx;
} iostream_t;

void iostream_init(iostream_t *iostream);

#endif
