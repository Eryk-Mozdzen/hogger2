#include <lrcp/stream.h>
#include <stm32u5xx_hal.h>

#include "serial.h"

#define RX_TIME_WATCHDOG    1000

#define MIN(a, b) ((a)<(b) ? (a) : (b))

static inline uint32_t fifo_pending(const serial_fifo_t *fifo) {
    return ((fifo->read>fifo->write ? sizeof(fifo->buffer) : 0) + fifo->write) - fifo->read;
}

static uint32_t reader(void *context, void *data, const uint32_t data_capacity) {
    serial_t *serial = context;
    uint32_t i;

    for(i=0; fifo_pending(&serial->fifo_rx) && i<data_capacity; i++) {
        ((uint8_t *)data)[i] = serial->fifo_rx.buffer[serial->fifo_rx.read];
        serial->fifo_rx.read++;
        serial->fifo_rx.read %=sizeof(serial->fifo_rx.buffer);
    }

    return i;
}

static uint32_t writer(void *context, const void *data, const uint32_t data_size) {
    serial_t *serial = context;

    for(uint32_t i=0; i<data_size; i++) {
        serial->fifo_tx.buffer[serial->fifo_tx.write] = ((const uint8_t *)data)[i];
        serial->fifo_tx.write++;
        serial->fifo_tx.write %=sizeof(serial->fifo_tx.buffer);
    }

    return data_size;
}

void serial_init(serial_t *serial, UART_HandleTypeDef *uart) {
    lrcp_stream_init(&serial->base, serial, reader, writer);

    serial->uart = uart;
    serial->time_last_rx = 0;
    serial->transmission = 0;

    serial->fifo_rx.read = 0;
    serial->fifo_rx.write = 0;
    serial->fifo_tx.read = 0;
    serial->fifo_tx.write = 0;

    HAL_UART_Receive_DMA(serial->uart, serial->fifo_rx.buffer, sizeof(serial->fifo_rx.buffer));
}

void serial_tick(serial_t *serial) {
    const uint32_t time = HAL_GetTick();
    const uint32_t rx_position = (sizeof(serial->fifo_rx.buffer) - __HAL_DMA_GET_COUNTER(serial->uart->hdmarx)) % sizeof(serial->fifo_rx.buffer);

    if(rx_position!=serial->fifo_rx.write) {
        serial->time_last_rx = time;
    }

    serial->fifo_rx.write = rx_position;

    if((time - serial->time_last_rx)>=RX_TIME_WATCHDOG) {
        serial->time_last_rx = time;
        HAL_UART_Receive_DMA(serial->uart, serial->fifo_rx.buffer, sizeof(serial->fifo_rx.buffer));
    }

    const uint32_t tx_pending = fifo_pending(&serial->fifo_tx);

    if(!serial->transmission && tx_pending) {
        const uint32_t len = MIN(tx_pending, sizeof(serial->fifo_tx.buffer) - serial->fifo_tx.read);

        serial->transmission = 1;
        HAL_UART_Transmit_DMA(serial->uart, &serial->fifo_tx.buffer[serial->fifo_tx.read], len);
        serial->fifo_tx.read +=len;
        serial->fifo_tx.read %=sizeof(serial->fifo_tx.buffer);
    }
}

void serial_transmit_callback(serial_t *serial, UART_HandleTypeDef *huart) {
    if(huart!=serial->uart) {
        return;
    }

    serial->transmission = 0;

    const uint32_t tx_pending = fifo_pending(&serial->fifo_tx);

    if(tx_pending) {
        const uint32_t len = MIN(tx_pending, sizeof(serial->fifo_tx.buffer) - serial->fifo_tx.read);

        serial->transmission = 1;
        HAL_UART_Transmit_DMA(serial->uart, &serial->fifo_tx.buffer[serial->fifo_tx.read], len);
        serial->fifo_tx.read +=len;
        serial->fifo_tx.read %=sizeof(serial->fifo_tx.buffer);
    }
}
