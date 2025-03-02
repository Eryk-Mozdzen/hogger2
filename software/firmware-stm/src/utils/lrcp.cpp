#include <lrcp/stream.h>

#include "utils/lrcp.hpp"

namespace lrcp {

Stream::Stream(void *buffer, const uint32_t capacity)
    : buffer{static_cast<uint8_t *>(buffer)}, capacity{capacity} {
    this->context = this;
    this->reader = Stream::read;
    this->writer = Stream::write;
    rd = 0;
    wr = 0;
}

uint32_t Stream::pending() const {
    return ((rd > wr ? capacity : 0) + wr) - rd;
}

uint32_t Stream::read(void *context, void *data, const uint32_t data_capacity) {
    Stream *stream = static_cast<Stream *>(context);

    uint32_t i;

    for(i = 0; stream->pending() && i < data_capacity; i++) {
        static_cast<uint8_t *>(data)[i] = stream->buffer[stream->rd];
        stream->rd++;
        stream->rd %= stream->capacity;
    }

    return i;
}

uint32_t Stream::write(void *context, const void *data, const uint32_t data_size) {
    Stream *stream = static_cast<Stream *>(context);

    for(uint32_t i = 0; i < data_size; i++) {
        stream->buffer[stream->wr] = static_cast<const uint8_t *>(data)[i];
        stream->wr++;
        stream->wr %= stream->capacity;
    }

    return data_size;
}

}
