#include <stdint.h>

#include "lrcp/stream.h"

void lrcp_stream_init(lrcp_stream_t *stream, void *context, const lrcp_stream_reader_t reader, const lrcp_stream_writer_t writer) {
    stream->context = context;
    stream->reader = reader;
    stream->writer = writer;
}

uint32_t lrcp_stream_read(lrcp_stream_t *stream, void *data, const uint32_t data_capacity) {
    if(stream->reader) {
        return stream->reader(stream->context, data, data_capacity);
    }

    return 0;
}

uint32_t lrcp_stream_write(lrcp_stream_t *stream, const void *data, const uint32_t data_size) {
    if(stream->writer) {
        return stream->writer(stream->context, data, data_size);
    }

    return 0;
}
