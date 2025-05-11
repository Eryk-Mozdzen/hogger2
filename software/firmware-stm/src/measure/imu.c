#include <main.h>
#include <stdint.h>
#include <stm32h5xx_hal.h>

#include "com/config.h"
#include "com/telemetry.h"
#include "generated/estimator.h"
#include "measure/mpu6050.h"
#include "utils/interrupt.h"
#include "utils/task.h"

extern I2C_HandleTypeDef hi2c2;

static volatile uint32_t ready;
static uint8_t buffer[14];

static float ascale[9] = {1, 0, 0, 0, 1, 0, 0, 0, 1};
static float aoffset[3] = {0};
static float goffset[3] = {0};

static float araw[3] = {0};
static float aout[3] = {0};
static float graw[3] = {0};
static float gout[3] = {0};

static void mpu6050_write(uint8_t address, uint8_t value) {
    HAL_I2C_Mem_Write(&hi2c2, MPU6050_ADDR << 1, address, 1, &value, 1, 100);
}

static void isr_data_ready() {
    HAL_I2C_Mem_Read_DMA(&hi2c2, MPU6050_ADDR << 1, MPU6050_REG_ACCEL_XOUT_H, 1, buffer,
                         sizeof(buffer));
}

static void isr_data_received(I2C_HandleTypeDef *hi2c) {
    (void)hi2c;

    ready = 1;
}

static void init() {
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

    HAL_I2C_RegisterCallback(&hi2c2, HAL_I2C_MEM_RX_COMPLETE_CB_ID, isr_data_received);
    interrupt_register(isr_data_ready, IMU_INT_Pin);
}

static void process() {
    {
        const int16_t x = (((int16_t)buffer[0]) << 8) | buffer[1];
        const int16_t y = (((int16_t)buffer[2]) << 8) | buffer[3];
        const int16_t z = (((int16_t)buffer[4]) << 8) | buffer[5];

        const float gain = 8192.f;
        const float g_to_ms2 = 9.80665f;

        araw[0] = -x * g_to_ms2 / gain;
        araw[1] = -y * g_to_ms2 / gain;
        araw[2] = +z * g_to_ms2 / gain;

        aout[0] = ascale[0] * araw[0] + ascale[1] * araw[1] + ascale[2] * araw[2] + aoffset[0];
        aout[1] = ascale[3] * araw[0] + ascale[4] * araw[1] + ascale[5] * araw[2] + aoffset[1];
        aout[2] = ascale[6] * araw[0] + ascale[7] * araw[1] + ascale[8] * araw[2] + aoffset[2];
    }

    {
        const int16_t x = (((int16_t)buffer[8]) << 8) | buffer[9];
        const int16_t y = (((int16_t)buffer[10]) << 8) | buffer[11];
        const int16_t z = (((int16_t)buffer[12]) << 8) | buffer[13];

        const float gain = 65.5f;
        const float dps_to_rads = 0.017453292519943f;

        graw[0] = -x * dps_to_rads / gain;
        graw[1] = -y * dps_to_rads / gain;
        graw[2] = +z * dps_to_rads / gain;

        gout[0] = graw[0] + goffset[0];
        gout[1] = graw[1] + goffset[1];
        gout[2] = graw[2] + goffset[2];
    }

    const float u[3] = {
        aout[0],
        aout[1],
        gout[2],
    };
    estimator_predict(u);
}

static void serialize_accel(cmp_ctx_t *cmp, void *context) {
    (void)context;

    cmp_write_map(cmp, 2);
    cmp_write_str(cmp, "raw", 3);
    cmp_write_array(cmp, 3);
    cmp_write_float(cmp, araw[0]);
    cmp_write_float(cmp, araw[1]);
    cmp_write_float(cmp, araw[2]);
    cmp_write_str(cmp, "out", 3);
    cmp_write_array(cmp, 3);
    cmp_write_float(cmp, aout[0]);
    cmp_write_float(cmp, aout[1]);
    cmp_write_float(cmp, aout[2]);
}

static void serialize_gyro(cmp_ctx_t *cmp, void *context) {
    (void)context;

    cmp_write_map(cmp, 2);
    cmp_write_str(cmp, "raw", 3);
    cmp_write_array(cmp, 3);
    cmp_write_float(cmp, graw[0]);
    cmp_write_float(cmp, graw[1]);
    cmp_write_float(cmp, graw[2]);
    cmp_write_str(cmp, "out", 3);
    cmp_write_array(cmp, 3);
    cmp_write_float(cmp, gout[0]);
    cmp_write_float(cmp, gout[1]);
    cmp_write_float(cmp, gout[2]);
}

TASK_REGISTER_INIT(init)
TASK_REGISTER_INTERRUPT(process, &ready)
TELEMETRY_REGISTER("accelerometer", serialize_accel, NULL)
TELEMETRY_REGISTER("gyroscope", serialize_gyro, NULL)
CONFIG_REGISTER("accelerometer_scale", ascale, 9)
CONFIG_REGISTER("accelerometer_offset", aoffset, 3)
CONFIG_REGISTER("gyroscope_offset", goffset, 3)
