#pragma once

#include <lrcp/stream.h>

namespace lrcp {

class Stream : public lrcp_stream_t {
    uint32_t pending() const;
    static uint32_t read(void *context, void *data, const uint32_t data_capacity);
    static uint32_t write(void *context, const void *data, const uint32_t data_size);

public:
    uint8_t *buffer;
    uint32_t capacity;
    uint32_t rd = 0;
    uint32_t wr = 0;

    Stream(void *buffer, const uint32_t capacity);
};

}
