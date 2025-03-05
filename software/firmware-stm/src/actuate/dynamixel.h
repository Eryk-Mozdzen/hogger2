#ifndef DYNAMIXEL_H
#define DYNAMIXEL_H

#include <stdint.h>
#include <stm32u5xx_hal.h>

#define DYNAMIXEL_REGISTER_MAX 4
#define DYNAMIXEL_QUEUE_MAX    128
#define DYNAMIXEL_TX_MAX       64
#define DYNAMIXEL_RX_MAX       64

typedef struct {
    uint8_t id;
    uint8_t led;
    float goal;
    float position;
    float velocity;
    float load;
    float voltage;
    float temperature;
    uint8_t error;
    uint32_t timestamp;
} dynamixel_servo_t;

typedef struct {
    uint8_t buffer[DYNAMIXEL_TX_MAX];
    uint8_t size;
    void *context;
} dynamixel_instruction_t;

typedef struct {
    UART_HandleTypeDef *uart;
    uint8_t status_buffer[DYNAMIXEL_RX_MAX];
    volatile uint8_t status_bytes;

    dynamixel_servo_t registered[DYNAMIXEL_REGISTER_MAX];
    dynamixel_instruction_t queue[DYNAMIXEL_QUEUE_MAX];
    uint8_t queue_r;
    uint8_t queue_w;
    volatile dynamixel_instruction_t *pending;
    uint32_t task_time;
    uint32_t start;
} dynamixel_t;

void dynamixel_init(dynamixel_t *dynamixel);
dynamixel_servo_t *dynamixel_register(dynamixel_t *dynamixel, const uint8_t id);
void dynamixel_tick(dynamixel_t *dynamixel);

void dynamixel_transmit_callback(dynamixel_t *dynamixel, const UART_HandleTypeDef *huart);
void dynamixel_receive_callback(dynamixel_t *dynamixel,
                                const UART_HandleTypeDef *huart,
                                const uint16_t size);

#endif
