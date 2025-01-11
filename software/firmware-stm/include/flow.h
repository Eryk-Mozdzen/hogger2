#ifndef FLOW_H
#define FLOW_H

#include <stdint.h>
#include <stm32u5xx_hal.h>

typedef struct {
    SPI_HandleTypeDef *spi;
    struct {
        GPIO_TypeDef *port;
        uint16_t pin;
    } chip_select;
    volatile uint32_t ready;
    uint32_t last_time;
    uint8_t buffer_tx[32];
    uint8_t buffer_rx[32];
    float velocity[2];
} flow_t;

void flow_init(flow_t *flow);
void flow_tick(flow_t *flow);

void flow_transmit_callback(flow_t *flow, const SPI_HandleTypeDef *hspi);

#endif
