#include <main.h>
#include <stdint.h>
#include <stm32u5xx_hal.h>

#include "com/telemetry.h"
#include "generated/estimator.h"
#include "measure/pmw3901.h"
#include "utils/task.h"

#define MIN_(x, y) (((x) < (y)) ? (x) : (y))
#define MAX_(x, y) (((x) > (y)) ? (x) : (y))

extern SPI_HandleTypeDef hspi2;

static volatile uint32_t ready;
static uint8_t buffer_tx[32];
static uint8_t buffer_rx[32];
static float velocity[2];

static void pmw3901_write(const uint8_t address, const uint8_t value) {
    uint8_t tx[] = {address | 0x80, value};

    HAL_GPIO_WritePin(FLOW_CS_GPIO_Port, FLOW_CS_Pin, GPIO_PIN_RESET);
    HAL_Delay(1);
    HAL_SPI_Transmit(&hspi2, tx, sizeof(tx), HAL_MAX_DELAY);
    HAL_Delay(1);
    HAL_GPIO_WritePin(FLOW_CS_GPIO_Port, FLOW_CS_Pin, GPIO_PIN_SET);
    HAL_Delay(1);
}

static uint8_t pmw3901_read(const uint8_t address) {
    uint8_t tx[] = {address & ~0x80, 0x00};
    uint8_t rx[] = {0x00, 0x00};

    HAL_GPIO_WritePin(FLOW_CS_GPIO_Port, FLOW_CS_Pin, GPIO_PIN_RESET);
    HAL_Delay(1);
    HAL_SPI_TransmitReceive(&hspi2, tx, rx, sizeof(tx), HAL_MAX_DELAY);
    HAL_Delay(1);
    HAL_GPIO_WritePin(FLOW_CS_GPIO_Port, FLOW_CS_Pin, GPIO_PIN_SET);
    HAL_Delay(1);

    return rx[1];
}

static void isr_transmit(SPI_HandleTypeDef *hspi) {
    (void)hspi;

    ready = 1;
}

static void init() {
    HAL_SPI_RegisterCallback(&hspi2, HAL_SPI_TX_RX_COMPLETE_CB_ID, isr_transmit);

    pmw3901_write(0x3A, 0x5A);

    HAL_Delay(5);

    // https://github.com/pimoroni/pmw3901-python/blob/main/pmw3901/__init__.py

    pmw3901_write(0x7F, 0x00);
    pmw3901_write(0x55, 0x01);
    pmw3901_write(0x50, 0x07);
    pmw3901_write(0x7F, 0x0E);
    pmw3901_write(0x43, 0x10);
    if(pmw3901_read(0x67) & 0b10000000) {
        pmw3901_write(0x48, 0x04);
    } else {
        pmw3901_write(0x48, 0x02);
    }
    pmw3901_write(0x7F, 0x00);
    pmw3901_write(0x51, 0x7B);
    pmw3901_write(0x50, 0x00);
    pmw3901_write(0x55, 0x00);
    pmw3901_write(0x7F, 0x0E);
    if(pmw3901_read(0x73) == 0x00) {
        uint8_t c1 = pmw3901_read(0x70);
        uint8_t c2 = pmw3901_read(0x71);
        if(c1 <= 28) {
            c1 += 14;
        }
        if(c1 > 28) {
            c1 += 11;
        }
        c1 = MAX_(0, MIN_(0x3F, c1));
        c2 = (c2 * 45);
        pmw3901_write(0x7F, 0x00);
        pmw3901_write(0x61, 0xAD);
        pmw3901_write(0x51, 0x70);
        pmw3901_write(0x7F, 0x0E);
        pmw3901_write(0x70, c1);
        pmw3901_write(0x71, c2);
    }

    pmw3901_write(0x7F, 0x00);
    pmw3901_write(0x61, 0xAD);
    pmw3901_write(0x7F, 0x03);
    pmw3901_write(0x40, 0x00);
    pmw3901_write(0x7F, 0x05);
    pmw3901_write(0x41, 0xB3);
    pmw3901_write(0x43, 0xF1);
    pmw3901_write(0x45, 0x14);
    pmw3901_write(0x5B, 0x32);
    pmw3901_write(0x5F, 0x34);
    pmw3901_write(0x7B, 0x08);
    pmw3901_write(0x7F, 0x06);
    pmw3901_write(0x44, 0x1B);
    pmw3901_write(0x40, 0xBF);
    pmw3901_write(0x4E, 0x3F);
    pmw3901_write(0x7F, 0x08);
    pmw3901_write(0x65, 0x20);
    pmw3901_write(0x6A, 0x18);
    pmw3901_write(0x7F, 0x09);
    pmw3901_write(0x4F, 0xAF);
    pmw3901_write(0x5F, 0x40);
    pmw3901_write(0x48, 0x80);
    pmw3901_write(0x49, 0x80);
    pmw3901_write(0x57, 0x77);
    pmw3901_write(0x60, 0x78);
    pmw3901_write(0x61, 0x78);
    pmw3901_write(0x62, 0x08);
    pmw3901_write(0x63, 0x50);
    pmw3901_write(0x7F, 0x0A);
    pmw3901_write(0x45, 0x60);
    pmw3901_write(0x7F, 0x00);
    pmw3901_write(0x4D, 0x11);
    pmw3901_write(0x55, 0x80);
    pmw3901_write(0x74, 0x21);
    pmw3901_write(0x75, 0x1F);
    pmw3901_write(0x4A, 0x78);
    pmw3901_write(0x4B, 0x78);
    pmw3901_write(0x44, 0x08);
    pmw3901_write(0x45, 0x50);
    pmw3901_write(0x64, 0xFF);
    pmw3901_write(0x65, 0x1F);
    pmw3901_write(0x7F, 0x14);
    pmw3901_write(0x65, 0x67);
    pmw3901_write(0x66, 0x08);
    pmw3901_write(0x63, 0x70);
    pmw3901_write(0x7F, 0x15);
    pmw3901_write(0x48, 0x48);
    pmw3901_write(0x7F, 0x07);
    pmw3901_write(0x41, 0x0D);
    pmw3901_write(0x43, 0x14);
    pmw3901_write(0x4B, 0x0E);
    pmw3901_write(0x45, 0x0F);
    pmw3901_write(0x44, 0x42);
    pmw3901_write(0x4C, 0x80);
    pmw3901_write(0x7F, 0x10);
    pmw3901_write(0x5B, 0x02);
    pmw3901_write(0x7F, 0x07);
    pmw3901_write(0x40, 0x41);
    pmw3901_write(0x70, 0x00);

    HAL_Delay(100);

    pmw3901_write(0x32, 0x44);
    pmw3901_write(0x7F, 0x07);
    pmw3901_write(0x40, 0x40);
    pmw3901_write(0x7F, 0x06);
    pmw3901_write(0x62, 0xF0);
    pmw3901_write(0x63, 0x00);
    pmw3901_write(0x7F, 0x0D);
    pmw3901_write(0x48, 0xC0);
    pmw3901_write(0x6F, 0xD5);
    pmw3901_write(0x7F, 0x00);
    pmw3901_write(0x5B, 0xA0);
    pmw3901_write(0x4E, 0xA8);
    pmw3901_write(0x5A, 0x50);
    pmw3901_write(0x40, 0x80);

    HAL_Delay(200);

    pmw3901_write(0x7F, 0x14);
    pmw3901_write(0x6F, 0x1C);
    pmw3901_write(0x7F, 0x00);
}

static void transmit() {
    memset(buffer_tx, 0, sizeof(buffer_tx));
    memset(buffer_rx, 0, sizeof(buffer_rx));

    for(uint8_t i = 0; i < 5; i++) {
        buffer_tx[2 * i] = (PMW3901_REG_MOTION + i) & ~0x80;
    }

    HAL_GPIO_WritePin(FLOW_CS_GPIO_Port, FLOW_CS_Pin, GPIO_PIN_RESET);
    HAL_SPI_TransmitReceive_IT(&hspi2, buffer_tx, buffer_rx, 10);
}

static void read() {
    HAL_GPIO_WritePin(FLOW_CS_GPIO_Port, FLOW_CS_Pin, GPIO_PIN_SET);

    uint8_t motion[5];
    for(uint8_t i = 0; i < sizeof(motion); i++) {
        motion[i] = buffer_rx[2 * i + 1];
    }

    const int16_t delta_x = (((int16_t)motion[2]) << 8) | motion[1];
    const int16_t delta_y = (((int16_t)motion[4]) << 8) | motion[3];

    const float tmp[2] = {
        -delta_y / (0.02f * PMW3901_FOCAL_LENGTH),
        -delta_x / (0.02f * PMW3901_FOCAL_LENGTH),
    };

    if((fabs(tmp[0]) < 100.f) && (fabs(tmp[1]) < 100.f)) {
        velocity[0] = tmp[0];
        velocity[1] = tmp[1];

        ESTIMATOR_CORRECT_FLOW(velocity);
    } else {
        velocity[0] = NAN;
        velocity[1] = NAN;
    }
}

static void serialize(cmp_ctx_t *cmp, void *context) {
    (void)context;

    cmp_write_array(cmp, 2);
    cmp_write_float(cmp, velocity[0]);
    cmp_write_float(cmp, velocity[1]);
}

TASK_REGISTER_INIT(init)
TASK_REGISTER_PERIODIC(transmit, 20000)
TASK_REGISTER_INTERRUPT(read, &ready)
TELEMETRY_REGISTER("optical_flow", serialize, NULL)
