#ifndef IMU_H
#define IMU_H

#include <stdint.h>
#include <stm32u5xx_hal.h>

typedef struct {
    I2C_HandleTypeDef *i2c;
    uint16_t interrupt;
    volatile uint32_t ready;
    uint8_t buffer[14];
    float acceleration[3];
    float gyration[3];
} imu_t;

void imu_init(imu_t *imu);
void imu_tick(imu_t *imu);

void imu_interrupt_callback(imu_t *imu, const uint16_t pin);
void imu_received_callback(imu_t *imu, const I2C_HandleTypeDef *hi2c);

#endif
