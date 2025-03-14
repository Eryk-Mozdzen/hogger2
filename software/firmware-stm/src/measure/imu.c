#include <main.h>
#include <stdint.h>
#include <stm32u5xx_hal.h>

#include "com/config.h"
#include "com/telemetry.h"
#include "generated/estimator.h"
#include "measure/mpu6050.h"
#include "utils/task.h"

extern I2C_HandleTypeDef hi2c1;

static volatile uint32_t ready;
static uint8_t buffer[14];

static float accel[3];
static float gyro[3];

static float calib_accel[12] = {0};
static float calib_gyro[3] = {0};

static void mpu6050_write(uint8_t address, uint8_t value) {
    HAL_I2C_Mem_Write(&hi2c1, MPU6050_ADDR << 1, address, 1, &value, 1, 100);
}

void HAL_GPIO_EXTI_Rising_Callback(uint16_t GPIO_Pin) {
    if(GPIO_Pin == IMU_INT_Pin) {
        HAL_I2C_Mem_Read_DMA(&hi2c1, MPU6050_ADDR << 1, MPU6050_REG_ACCEL_XOUT_H, 1, buffer,
                             sizeof(buffer));
    }
}

static void isr_memory_received(I2C_HandleTypeDef *hi2c) {
    (void)hi2c;

    ready = 1;
}

static void init() {
    HAL_I2C_RegisterCallback(&hi2c1, HAL_I2C_MEM_RX_COMPLETE_CB_ID, isr_memory_received);

    mpu6050_write(MPU6050_REG_PWR_MGMT_1, MPU6050_PWR_MGMT_1_DEVICE_RESET);

    HAL_Delay(10);

    mpu6050_write(MPU6050_REG_SIGNAL_PATH_RESET, MPU6050_SIGNAL_PATH_RESET_GYRO |
                                                     MPU6050_SIGNAL_PATH_RESET_ACCEL |
                                                     MPU6050_SIGNAL_PATH_RESET_TEMP);

    HAL_Delay(10);

    mpu6050_write(MPU6050_REG_INT_ENABLE, MPU6050_INT_ENABLE_FIFO_OVERLOW_DISABLE |
                                              MPU6050_INT_ENABLE_I2C_MST_INT_DISABLE |
                                              MPU6050_INT_ENABLE_DATA_RDY_ENABLE);

    mpu6050_write(MPU6050_REG_INT_PIN_CFG,
                  MPU6050_INT_PIN_CFG_LEVEL_ACTIVE_HIGH | MPU6050_INT_PIN_CFG_PUSH_PULL |
                      MPU6050_INT_PIN_CFG_PULSE | MPU6050_INT_PIN_CFG_STATUS_CLEAR_AFTER_ANY |
                      MPU6050_INT_PIN_CFG_FSYNC_DISABLE | MPU6050_INT_PIN_CFG_I2C_BYPASS_DISABLE);

    mpu6050_write(MPU6050_REG_PWR_MGMT_1,
                  MPU6050_PWR_MGMT_1_TEMP_DIS | MPU6050_PWR_MGMT_1_CLOCK_INTERNAL);

    mpu6050_write(MPU6050_REG_CONFIG,
                  MPU6050_CONFIG_EXT_SYNC_DISABLED | MPU6050_CONFIG_DLPF_SETTING_6);

    mpu6050_write(MPU6050_REG_ACCEL_CONFIG, MPU6050_ACCEL_CONFIG_RANGE_4G);

    mpu6050_write(MPU6050_REG_GYRO_CONFIG, MPU6050_GYRO_CONFIG_RANGE_500DPS);

    mpu6050_write(MPU6050_REG_SMPLRT_DIV, 0);
}

static void read() {
    {
        const int16_t raw_x = (((int16_t)buffer[8]) << 8) | buffer[9];
        const int16_t raw_y = (((int16_t)buffer[10]) << 8) | buffer[11];
        const int16_t raw_z = (((int16_t)buffer[12]) << 8) | buffer[13];

        const float gain = 65.5f;
        const float dps_to_rads = 0.017453292519943f;

        gyro[0] = -raw_x * dps_to_rads / gain;
        gyro[1] = -raw_y * dps_to_rads / gain;
        gyro[2] = +raw_z * dps_to_rads / gain;
    }

    {
        const int16_t raw_x = (((int16_t)buffer[0]) << 8) | buffer[1];
        const int16_t raw_y = (((int16_t)buffer[2]) << 8) | buffer[3];
        const int16_t raw_z = (((int16_t)buffer[4]) << 8) | buffer[5];

        const float gain = 8192.f;
        const float g_to_ms2 = 9.80665f;

        accel[0] = -raw_x * g_to_ms2 / gain;
        accel[1] = -raw_y * g_to_ms2 / gain;
        accel[2] = +raw_z * g_to_ms2 / gain;
    }

    const float u[3] = {
        accel[0],
        accel[1],
        gyro[2],
    };
    ESTIMATOR_PREDICT(u);
}

static void serialize_accel(cmp_ctx_t *cmp, void *context) {
    (void)context;

    cmp_write_array(cmp, 3);
    cmp_write_float(cmp, accel[0]);
    cmp_write_float(cmp, accel[1]);
    cmp_write_float(cmp, accel[2]);
}

static void serialize_gyro(cmp_ctx_t *cmp, void *context) {
    (void)context;

    cmp_write_array(cmp, 3);
    cmp_write_float(cmp, gyro[0]);
    cmp_write_float(cmp, gyro[1]);
    cmp_write_float(cmp, gyro[2]);
}

TASK_REGISTER_INIT(init)
TASK_REGISTER_INTERRUPT(read, &ready)
TELEMETRY_REGISTER("accelerometer", serialize_accel, NULL)
TELEMETRY_REGISTER("gyroscope", serialize_gyro, NULL)
CONFIG_REGISTER("accelerometer", calib_accel, 12)
CONFIG_REGISTER("gyroscope", calib_gyro, 3)
