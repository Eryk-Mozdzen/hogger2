#include <main.h>
#include <stdint.h>
#include <stm32h5xx_hal.h>

#include "com/config.h"
#include "com/telemetry.h"
#include "generated/estimator.h"
#include "measure/qmc5883l.h"
#include "utils/interrupt.h"
#include "utils/task.h"

extern I2C_HandleTypeDef hi2c1;

static volatile uint32_t ready;
static uint8_t buffer[6];

static float mag[3];
static float calib[12] = {0};

static void hmc5883l_write(uint8_t address, uint8_t value) {
    HAL_I2C_Mem_Write(&hi2c1, QMC5883L_ADDR << 1, address, 1, &value, 1, 100);
}

static void isr_data_ready() {
    HAL_I2C_Mem_Read_DMA(&hi2c1, QMC5883L_ADDR << 1, QMC5883L_REG_DATA_OUTPUT_X_LSB, 1, buffer,
                         sizeof(buffer));
}

static void isr_data_received(I2C_HandleTypeDef *hi2c) {
    (void)hi2c;

    ready = 1;
}

static void init() {
    interrupt_register(isr_data_ready, MAG_INT_Pin);
    HAL_I2C_RegisterCallback(&hi2c1, HAL_I2C_MEM_RX_COMPLETE_CB_ID, isr_data_received);

    hmc5883l_write(QMC5883L_REG_CONTROL_2, QMC5883L_CONFIG_2_SOFT_RST);

    HAL_Delay(100);

    hmc5883l_write(QMC5883L_REG_SET_RESET, QMC5883L_SET_RESET_RECOMMENDED);

    hmc5883l_write(QMC5883L_REG_CONTROL_2, QMC5883L_CONFIG_2_INT_ENB_ENABLE);

    hmc5883l_write(QMC5883L_REG_CONTROL_1, QMC5883L_CONFIG_1_OSR_512 | QMC5883L_CONFIG_1_RNG_8G |
                                               QMC5883L_CONFIG_1_ODR_200HZ |
                                               QMC5883L_CONFIG_1_MODE_CONTINOUS);
}

static void read() {
    const int16_t raw_x = (((int16_t)buffer[1]) << 8) | buffer[0];
    const int16_t raw_y = (((int16_t)buffer[3]) << 8) | buffer[2];
    const int16_t raw_z = (((int16_t)buffer[5]) << 8) | buffer[4];

    const float gain = 1.f / 3000.f;

    mag[0] = +raw_x * gain;
    mag[1] = -raw_y * gain;
    mag[2] = -raw_z * gain;

    ESTIMATOR_CORRECT_FLOW(mag);
}

static void serialize(cmp_ctx_t *cmp, void *context) {
    (void)context;

    cmp_write_array(cmp, 3);
    cmp_write_float(cmp, mag[0]);
    cmp_write_float(cmp, mag[1]);
    cmp_write_float(cmp, mag[2]);
}

TASK_REGISTER_INIT(init)
TASK_REGISTER_INTERRUPT(read, &ready)
TELEMETRY_REGISTER("magnetometer", serialize, NULL)
CONFIG_REGISTER("magnetometer", calib, 12)
