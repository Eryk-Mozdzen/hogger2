#ifndef LRCP_STREAM_H
#define LRCP_STREAM_H

#include <stdint.h>

typedef uint32_t (*lrcp_stream_reader_t)(void *, void *, const uint32_t);
typedef uint32_t (*lrcp_stream_writer_t)(void *, const void *, const uint32_t);

typedef struct {
    void *context;
    lrcp_stream_reader_t reader;
    lrcp_stream_writer_t writer;
} lrcp_stream_t;

void lrcp_stream_init(lrcp_stream_t *stream, void *context, const lrcp_stream_reader_t reader, const lrcp_stream_writer_t writer);
uint32_t lrcp_stream_read(lrcp_stream_t *stream, void *data, const uint32_t data_capacity);
uint32_t lrcp_stream_write(lrcp_stream_t *stream, const void *data, const uint32_t data_size);

#endif
