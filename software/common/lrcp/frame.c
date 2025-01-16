#include <stdbool.h>

#include "lrcp/stream.h"
#include "lrcp/frame.h"

enum decoder_state {
    DECODER_STATE_RESET,
    DECODER_STATE_PROCESS,
};

static void crc32(uint32_t *crc, const void *buffer, const uint32_t size) {
    for(uint32_t i=0; i<size; i++) {
        *crc ^=((uint8_t *)buffer)[i];
        for(uint8_t j=0; j<8; j++) {
            if(*crc & 0x00000001) {
                *crc = (*crc>>1)^0xEDB88320;
            } else {
                *crc >>=1;
            }
        }
    }
}

static bool cobs_encode(lrcp_stream_t *ostream, uint8_t **accumulator, uint8_t **code, const void *buffer, const uint32_t size) {
    for(uint32_t i=0; i<size; i++) {
        const uint8_t byte = ((uint8_t *)buffer)[i];

        if(byte) {
            **accumulator = byte;
            (*accumulator)++;
            (**code)++;
        }

        if(!byte || (**code==0xFF)) {
            if(lrcp_stream_write(ostream, *code, **code)!=(**code)) {
                return false;
            }
            *accumulator = *code;
            (*accumulator)++;
            **code = 1;
        }
    }

    return true;
}

bool lrcp_frame_encode(lrcp_stream_t *ostream, const void *payload, const uint32_t payload_size) {
    uint32_t crc = 0;
    crc32(&crc, payload, payload_size);

    uint8_t block[256];
    uint8_t *code = &block[0];
    uint8_t *accumulator = &block[1];

    *code = 1;

    if(!cobs_encode(ostream, &accumulator, &code, payload, payload_size)) {
        return false;
    }
    if(!cobs_encode(ostream, &accumulator, &code, &crc, sizeof(crc))) {
        return false;
    }
    if(lrcp_stream_write(ostream, code, *code)!=(*code)) {
        return false;
    }
    const uint8_t terminator = 0;
    if(lrcp_stream_write(ostream, &terminator, sizeof(terminator))!=sizeof(terminator)) {
        return false;
    }

    return true;
}

void lrcp_frame_decoder_init(lrcp_decoder_t *decoder) {
    decoder->state = DECODER_STATE_RESET;
}

uint32_t lrcp_frame_decode(lrcp_stream_t *istream, lrcp_decoder_t *decoder, void *payload, const uint32_t payload_capacity) {
    while(1) {
        switch(decoder->state) {
            case DECODER_STATE_RESET: {
                decoder->block = 0;
                decoder->code = 0xFF;
                decoder->decode = payload;
                decoder->crc = 0;
                decoder->state = DECODER_STATE_PROCESS;
            } break;
            case DECODER_STATE_PROCESS: {
                uint8_t byte;
                if(lrcp_stream_read(istream, &byte, sizeof(byte))!=sizeof(byte)) {
                    return 0;
                }

                if(decoder->block) {
                    *decoder->decode = byte;
                    crc32(&decoder->crc, decoder->decode, 1);
                    decoder->decode++;

                    if(decoder->decode>(((uint8_t *)payload)+payload_capacity)) {
                        decoder->state = DECODER_STATE_RESET;
                        const uint32_t num = (decoder->decode - ((uint8_t *)payload));
                        if(!decoder->crc && (num>=sizeof(decoder->crc))) {
                            return (num - sizeof(decoder->crc));
                        } else {
                            return 0;
                        }
                    }
                } else {
                    decoder->block = byte;
                    if(decoder->block && (decoder->code!=0xFF)) {
                        *decoder->decode = 0x00;
                        crc32(&decoder->crc, decoder->decode, 1);
                        decoder->decode++;

                        if(decoder->decode>(((uint8_t *)payload)+payload_capacity)) {
                            decoder->state = DECODER_STATE_RESET;
                            const uint32_t num = (decoder->decode - ((uint8_t *)payload));
                            if(!decoder->crc && (num>=sizeof(decoder->crc))) {
                                return (num - sizeof(decoder->crc));
                            } else {
                                return 0;
                            }
                        }
                    }

                    decoder->code = decoder->block;

                    if(!decoder->code) {
                        decoder->state = DECODER_STATE_RESET;
                        const uint32_t num = (decoder->decode - ((uint8_t *)payload));
                        if(!decoder->crc && (num>=sizeof(decoder->crc))) {
                            return (num - sizeof(decoder->crc));
                        } else {
                            return 0;
                        }
                    }
                }

                decoder->block--;
            } break;
        }
    }

    return 0;
}
