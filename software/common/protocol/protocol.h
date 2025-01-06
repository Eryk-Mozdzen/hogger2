#ifndef PROTOCOL_H
#define PROTOCOL_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stddef.h>
#include <stdint.h>
#include <stdbool.h>

#define PROTOCOL_INIT {NULL, NULL, {NULL, 0, 0, 0}, {NULL, 0, 0, 0}, true, 0, 0, 0, NULL, NULL, 0, 0, 0, 0, 0}

typedef void (*protocol_tx_cb_t)(const void *, const uint32_t);
typedef void (*protocol_rx_cb_t)(const uint8_t, const void *, const uint32_t);

typedef struct {
    uint8_t *buffer;
    uint32_t size;
    uint32_t read;
    uint32_t write;
} protocol_fifo_t;

typedef struct {
    protocol_tx_cb_t callback_tx;
    protocol_rx_cb_t callback_rx;
    protocol_fifo_t fifo_tx;
    protocol_fifo_t fifo_rx;
    bool available;
    uint32_t time;

    uint32_t time_last_tx;
    uint32_t time_last_rx;
    uint8_t *decoded;
    uint8_t *cursor;
    uint32_t max;
    uint8_t cobs;
    uint8_t counter;
    uint8_t state;
    uint32_t crc;
} protocol_t;

void fifo_write(protocol_fifo_t *fifo, const uint8_t byte);
uint8_t fifo_read(protocol_fifo_t *fifo);
uint32_t fifo_pending(const protocol_fifo_t *fifo) ;

void protocol_enqueue(protocol_t *instance, const uint8_t id, const void *payload, const uint32_t size);
void protocol_process(protocol_t *instance);

#ifdef __cplusplus
}
#endif

#endif
