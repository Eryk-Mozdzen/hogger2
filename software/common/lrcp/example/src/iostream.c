#include <stdio.h>
#include <lrcp/stream.h>

#include "iostream.h"

static uint32_t reader(void *context, void *data, const uint32_t data_capacity) {
    iostream_t *iostream = context;

    uint32_t data_size = 0;
    while(1) {
        if(data_size>=data_capacity) {
            return data_size;
        }

        if((((iostream->rx>iostream->tx ? sizeof(iostream->buffer) : 0) + iostream->tx) - iostream->rx)==0) {
            return data_size;
        }

        ((uint8_t *)data)[data_size] = iostream->buffer[iostream->rx];
        iostream->rx++;
        iostream->rx %=sizeof(iostream->buffer);

        data_size++;
    }
}

static uint32_t writer(void *context, const void *data, const uint32_t data_size) {
    iostream_t *iostream = context;

    for(uint32_t i=0; i<data_size; i++) {
        const uint8_t byte = ((uint8_t *)data)[i];

        iostream->buffer[iostream->tx] = byte;
        iostream->tx++;
        iostream->tx %=sizeof(iostream->buffer);

        printf("%02X ", byte);
    }

    return data_size;
}

void iostream_init(iostream_t *iostream) {
    lrcp_stream_init(&iostream->base, iostream, reader, writer);

    iostream->rx = 0;
    iostream->tx = 0;
}
