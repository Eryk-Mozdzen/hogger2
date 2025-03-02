#ifndef LRCP_FRAME_H
#define LRCP_FRAME_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdbool.h>

#include "lrcp/stream.h"

typedef struct {
    uint8_t state;
    uint8_t block;
    uint8_t code;
    uint8_t *decode;
    uint32_t crc;
} lrcp_decoder_t;

bool lrcp_frame_encode(lrcp_stream_t *ostream, const void *payload, const uint32_t payload_size);
void lrcp_frame_decoder_init(lrcp_decoder_t *decoder);
uint32_t lrcp_frame_decode(lrcp_stream_t *istream, lrcp_decoder_t *decoder, void *payload, const uint32_t payload_capacity);

#ifdef __cplusplus
}
#endif

#endif
