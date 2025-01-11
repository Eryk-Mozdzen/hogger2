#include <stdint.h>
#include <stm32u5xx_hal.h>

#include "flow.h"
#include "pmw3901_regs.h"

void pmw3901_write(flow_t *flow, const uint8_t address, const uint8_t value) {
	uint8_t tx[] = {address | 0x80, value};

	HAL_GPIO_WritePin(flow->chip_select.port, flow->chip_select.pin, GPIO_PIN_RESET);
	HAL_Delay(1);
	HAL_SPI_Transmit(flow->spi, tx, sizeof(tx), HAL_MAX_DELAY);
	HAL_Delay(1);
	HAL_GPIO_WritePin(flow->chip_select.port, flow->chip_select.pin, GPIO_PIN_SET);
	HAL_Delay(1);
}

void flow_init(flow_t *flow) {
    pmw3901_write(flow, 0x3A, 0x5A);

	HAL_Delay(5);

	pmw3901_write(flow, 0x7F, 0x00);
	pmw3901_write(flow, 0x61, 0xAD);
	pmw3901_write(flow, 0x7F, 0x03);
	pmw3901_write(flow, 0x40, 0x00);
	pmw3901_write(flow, 0x7F, 0x05);
	pmw3901_write(flow, 0x41, 0xB3);
	pmw3901_write(flow, 0x43, 0xF1);
	pmw3901_write(flow, 0x45, 0x14);
	pmw3901_write(flow, 0x5B, 0x32);
	pmw3901_write(flow, 0x5F, 0x34);
	pmw3901_write(flow, 0x7B, 0x08);
	pmw3901_write(flow, 0x7F, 0x06);
	pmw3901_write(flow, 0x44, 0x1B);
	pmw3901_write(flow, 0x40, 0xBF);
	pmw3901_write(flow, 0x4E, 0x3F);
	pmw3901_write(flow, 0x7F, 0x08);
	pmw3901_write(flow, 0x65, 0x20);
	pmw3901_write(flow, 0x6A, 0x18);
	pmw3901_write(flow, 0x7F, 0x09);
	pmw3901_write(flow, 0x4F, 0xAF);
	pmw3901_write(flow, 0x5F, 0x40);
	pmw3901_write(flow, 0x48, 0x80);
	pmw3901_write(flow, 0x49, 0x80);
	pmw3901_write(flow, 0x57, 0x77);
	pmw3901_write(flow, 0x60, 0x78);
	pmw3901_write(flow, 0x61, 0x78);
	pmw3901_write(flow, 0x62, 0x08);
	pmw3901_write(flow, 0x63, 0x50);
	pmw3901_write(flow, 0x7F, 0x0A);
	pmw3901_write(flow, 0x45, 0x60);
	pmw3901_write(flow, 0x7F, 0x00);
	pmw3901_write(flow, 0x4D, 0x11);
	pmw3901_write(flow, 0x55, 0x80);
	pmw3901_write(flow, 0x74, 0x1F);
	pmw3901_write(flow, 0x75, 0x1F);
	pmw3901_write(flow, 0x4A, 0x78);
	pmw3901_write(flow, 0x4B, 0x78);
	pmw3901_write(flow, 0x44, 0x08);
	pmw3901_write(flow, 0x45, 0x50);
	pmw3901_write(flow, 0x64, 0xFF);
	pmw3901_write(flow, 0x65, 0x1F);
	pmw3901_write(flow, 0x7F, 0x14);
	pmw3901_write(flow, 0x65, 0x60);
	pmw3901_write(flow, 0x66, 0x08);
	pmw3901_write(flow, 0x63, 0x78);
	pmw3901_write(flow, 0x7F, 0x15);
	pmw3901_write(flow, 0x48, 0x58);
	pmw3901_write(flow, 0x7F, 0x07);
	pmw3901_write(flow, 0x41, 0x0D);
	pmw3901_write(flow, 0x43, 0x14);
	pmw3901_write(flow, 0x4B, 0x0E);
	pmw3901_write(flow, 0x45, 0x0F);
	pmw3901_write(flow, 0x44, 0x42);
	pmw3901_write(flow, 0x4C, 0x80);
	pmw3901_write(flow, 0x7F, 0x10);
	pmw3901_write(flow, 0x5B, 0x02);
	pmw3901_write(flow, 0x7F, 0x07);
	pmw3901_write(flow, 0x40, 0x41);
	pmw3901_write(flow, 0x70, 0x00);

	HAL_Delay(100);

	pmw3901_write(flow, 0x32, 0x44);
	pmw3901_write(flow, 0x7F, 0x07);
	pmw3901_write(flow, 0x40, 0x40);
	pmw3901_write(flow, 0x7F, 0x06);
	pmw3901_write(flow, 0x62, 0xf0);
	pmw3901_write(flow, 0x63, 0x00);
	pmw3901_write(flow, 0x7F, 0x0D);
	pmw3901_write(flow, 0x48, 0xC0);
	pmw3901_write(flow, 0x6F, 0xd5);
	pmw3901_write(flow, 0x7F, 0x00);
	pmw3901_write(flow, 0x5B, 0xa0);
	pmw3901_write(flow, 0x4E, 0xA8);
	pmw3901_write(flow, 0x5A, 0x50);
	pmw3901_write(flow, 0x40, 0x80);
}

void flow_tick(flow_t *flow) {
    const uint32_t time = HAL_GetTick();

    if((time - flow->last_time)>=20) {
        flow->last_time = time;

        for(uint8_t i=0; i<5; i++) {
            flow->buffer_tx[2*i] = (PMW3901_REG_MOTION + i) & ~0x80;
        }

        HAL_GPIO_WritePin(flow->chip_select.port, flow->chip_select.pin, GPIO_PIN_RESET);
        HAL_SPI_TransmitReceive_IT(flow->spi, flow->buffer_tx, flow->buffer_rx, 10);
    }

    if(flow->ready) {
        flow->ready = 0;

        HAL_GPIO_WritePin(flow->chip_select.port, flow->chip_select.pin, GPIO_PIN_SET);

        uint8_t motion[5];
        for(uint8_t i=0; i<sizeof(motion); i++) {
            motion[i] = flow->buffer_rx[2*i + 1];
        }

        const int16_t delta_x = (((int16_t)motion[2])<<8) | motion[1];
        const int16_t delta_y = (((int16_t)motion[4])<<8) | motion[3];

        const float tmp[2] = {
            -delta_y/(0.02f*PMW3901_FOCAL_LENGTH),
            -delta_x/(0.02f*PMW3901_FOCAL_LENGTH),
        };

        if((fabs(tmp[0])<7.4f) && (fabs(tmp[1])<7.4f)) {
            flow->velocity[0] = tmp[0];
            flow->velocity[1] = tmp[1];
        } else {
            flow->velocity[0] = NAN;
            flow->velocity[1] = NAN;
        }
    }
}

void flow_transmit_callback(flow_t *flow, const SPI_HandleTypeDef *hspi) {
    if(hspi==flow->spi) {
		flow->ready = 1;
	}
}
