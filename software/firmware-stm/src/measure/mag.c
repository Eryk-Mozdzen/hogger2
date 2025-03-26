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

static float scale[9] = {1, 0, 0, 0, 1, 0, 0, 0, 1};
static float offset[3] = {0};

static float raw[3] = {0};
static float out[3] = {0};

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

static void process() {
    const int16_t x = (((int16_t)buffer[1]) << 8) | buffer[0];
    const int16_t y = (((int16_t)buffer[3]) << 8) | buffer[2];
    const int16_t z = (((int16_t)buffer[5]) << 8) | buffer[4];

    const float gain = 1.f / 3000.f;

    raw[0] = +x * gain;
    raw[1] = -y * gain;
    raw[2] = -z * gain;

    out[0] = scale[0] * raw[0] + scale[1] * raw[1] + scale[2] * raw[2] + offset[0];
    out[1] = scale[3] * raw[0] + scale[4] * raw[1] + scale[5] * raw[2] + offset[1];
    out[2] = scale[6] * raw[0] + scale[7] * raw[1] + scale[8] * raw[2] + offset[2];

    const float len = sqrtf(out[0] * out[0] + out[1] * out[1] + out[2] * out[2]);
    const float normalized[3] = {
        out[0] / len,
        out[1] / len,
        out[2] / len,
    };

    ESTIMATOR_CORRECT_MAGNETOMETER(normalized);
}

static void serialize(cmp_ctx_t *cmp, void *context) {
    (void)context;

    cmp_write_map(cmp, 2);
    cmp_write_str(cmp, "raw", 3);
    cmp_write_array(cmp, 3);
    cmp_write_float(cmp, raw[0]);
    cmp_write_float(cmp, raw[1]);
    cmp_write_float(cmp, raw[2]);
    cmp_write_str(cmp, "out", 3);
    cmp_write_array(cmp, 3);
    cmp_write_float(cmp, out[0]);
    cmp_write_float(cmp, out[1]);
    cmp_write_float(cmp, out[2]);
}

TASK_REGISTER_INIT(init)
TASK_REGISTER_INTERRUPT(process, &ready)
TELEMETRY_REGISTER("magnetometer", serialize, NULL)
CONFIG_REGISTER("magnetometer_scale", scale, 9)
CONFIG_REGISTER("magnetometer_offset", offset, 3)
