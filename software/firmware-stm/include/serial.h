#ifndef SERIAL_H
#define SERIAL_H

#include <lrcp/stream.h>
#include <stm32u5xx_hal.h>

typedef struct {
    uint8_t buffer[10*1024];
    volatile uint32_t read;
    volatile uint32_t write;
} serial_fifo_t;

typedef struct {
    lrcp_stream_t base;
    UART_HandleTypeDef *uart;
    serial_fifo_t fifo_rx;
    serial_fifo_t fifo_tx;
    volatile uint32_t transmission;
    uint32_t time_last_rx;
} serial_t;

void serial_init(serial_t *serial, UART_HandleTypeDef *uart);
void serial_tick(serial_t *serial);

void serial_transmit_callback(serial_t *serial, UART_HandleTypeDef *huart);

#endif
