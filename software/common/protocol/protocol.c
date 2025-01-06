#include <stddef.h>
#include <stdint.h>
#include <string.h>

#include "protocol.h"

#define TX_SIZE_THRESHOLD   256
#define TX_TIME_THRESHOLD   100

#define MIN(a, b)           ((a)<(b) ? (a) : (b))

enum state {
    STATE_START,
    STATE_DATA,
};

void fifo_write(protocol_fifo_t *fifo, const uint8_t byte) {
    fifo->buffer[fifo->write] = byte;
    fifo->write++;
    fifo->write %=fifo->size;
}

uint8_t fifo_read(protocol_fifo_t *fifo) {
    const uint8_t byte = fifo->buffer[fifo->read];
    fifo->read++;
    fifo->read %=fifo->size;
    return byte;
}

uint32_t fifo_pending(const protocol_fifo_t *fifo) {
    return ((fifo->read>fifo->write ? fifo->size : 0) + fifo->write) - fifo->read;
}

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

static uint8_t * cobs_encode(protocol_fifo_t *fifo, uint8_t *cobs, const void *buffer, const uint32_t size) {
    for(uint32_t i=0; i<size; i++) {
        const uint8_t byte = ((uint8_t *)buffer)[i];

        if(*cobs==0xFF) {
            cobs = &fifo->buffer[fifo->write];
            fifo_write(fifo, 1);
        }

        if(byte) {
            (*cobs)++;
            fifo_write(fifo, byte);
        } else {
            cobs = &fifo->buffer[fifo->write];
            fifo_write(fifo, 1);
        }
    }

    return cobs;
}

void protocol_enqueue(protocol_t *instance, const uint8_t id, const void *payload, const uint32_t size) {
    uint32_t crc = 0;
    crc32(&crc, &id, sizeof(id));
    crc32(&crc, payload, size);

    uint8_t *cobs = &instance->fifo_tx.buffer[instance->fifo_tx.write];
    fifo_write(&instance->fifo_tx, 1);

    cobs = cobs_encode(&instance->fifo_tx, cobs, &id, sizeof(id));
    cobs = cobs_encode(&instance->fifo_tx, cobs, payload, size);
    cobs = cobs_encode(&instance->fifo_tx, cobs, &crc, sizeof(crc));

    fifo_write(&instance->fifo_tx, 0);
}

void protocol_process(protocol_t *instance) {
    while(fifo_pending(&instance->fifo_rx)>0) {
        const uint8_t byte = fifo_read(&instance->fifo_rx);

        switch(instance->state) {
            case STATE_START: {
                instance->cursor = instance->decoded;
                instance->cobs = byte;
                instance->crc = 0;
                instance->counter = 0;
                if(byte) {
                    instance->state = STATE_DATA;
                }
            } break;
            case STATE_DATA: {
                instance->counter++;

                if(!byte) {
                    const uint32_t num = instance->cursor - instance->decoded;

                    if(!instance->crc && num>=5) {
                        const uint8_t id = instance->decoded[0];
                        const uint8_t *payload = &instance->decoded[1];
                        const uint32_t size = num - 5;

                        instance->callback_rx(id, payload, size);
                        instance->time_last_rx = instance->time;
                    }

                    instance->state = STATE_START;
                } else if(instance->cobs==instance->counter) {
                    if(instance->cobs!=0xFF) {
                        *instance->cursor = 0;
                        crc32(&instance->crc, instance->cursor, 1);
                        instance->cursor++;
                        if(instance->cursor>=instance->decoded+instance->max) {
                            instance->state = STATE_START;
                        }
                    }
                    instance->cobs = byte;
                    instance->counter = 0;
                } else {
                    *instance->cursor = byte;
                    crc32(&instance->crc, instance->cursor, 1);
                    instance->cursor++;
                    if(instance->cursor>=instance->decoded+instance->max) {
                        instance->state = STATE_START;
                    }
                }
            } break;
        }
    }

    if(instance->callback_tx) {
        const uint32_t delta_time = instance->time - instance->time_last_tx;
        const uint32_t tx_pending = fifo_pending(&instance->fifo_tx);

        if(instance->available && tx_pending && (tx_pending>TX_SIZE_THRESHOLD || delta_time>TX_TIME_THRESHOLD)) {
            const uint32_t len = MIN(tx_pending, instance->fifo_tx.size - instance->fifo_tx.read);

            instance->callback_tx(&instance->fifo_tx.buffer[instance->fifo_tx.read], len);
            instance->fifo_tx.read +=len;
            instance->fifo_tx.read %=instance->fifo_tx.size;

            instance->time_last_tx = instance->time;
        }
    }
}
