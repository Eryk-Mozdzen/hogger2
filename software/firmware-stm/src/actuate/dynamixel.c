#include <stdint.h>
#include <stm32u5xx_hal.h>
#include <string.h>

#include "actuate/ax12a.h"
#include "actuate/dynamixel.h"

#define PI 3.141592653589f

#define UNREGISTRED 0xFF

typedef enum {
    INSTRUCTION_PING = 0x01,
    INSTRUCTION_READ = 0x02,
    INSTRUCTION_WRITE = 0x03,
    INSTRUCTION_REG_WRITE = 0x04,
    INSTRUCTION_ACTION = 0x05,
    INSTRUCTION_FACTORY_RESET = 0x06,
    INSTRUCTION_REBOOT = 0x08,
    INSTRUCTION_SYNC_WRITE = 0x83,
    INSTRUCTION_BULK_READ = 0x92,
} instruction_t;

static void push_instruction_packet(dynamixel_t *dynamixel,
                                    const uint8_t id,
                                    const instruction_t instruction,
                                    const uint8_t *params,
                                    const uint8_t params_size,
                                    void *context) {
    dynamixel_instruction_t *instr = &dynamixel->queue[dynamixel->queue_w];
    dynamixel->queue_w++;
    dynamixel->queue_w %= DYNAMIXEL_QUEUE_MAX;

    const uint8_t length = params_size + 2;

    instr->buffer[0] = 0xFF;
    instr->buffer[1] = 0xFF;
    instr->buffer[2] = id;
    instr->buffer[3] = length;
    instr->buffer[4] = instruction;

    uint8_t checksum = 0;
    checksum += id;
    checksum += length;
    checksum += instruction;

    for(uint8_t i = 0; i < params_size; i++) {
        instr->buffer[i + 5] = params[i];
        checksum += params[i];
    }

    instr->buffer[params_size + 5] = ~checksum;

    instr->size = params_size + 6;
    instr->context = context;
}

static dynamixel_instruction_t *pop_instruction_packet(dynamixel_t *dynamixel) {
    if(dynamixel->queue_r == dynamixel->queue_w) {
        return NULL;
    }

    dynamixel_instruction_t *instr = &dynamixel->queue[dynamixel->queue_r];

    dynamixel->queue_r++;
    dynamixel->queue_r %= DYNAMIXEL_QUEUE_MAX;

    return instr;
}

void dynamixel_init(dynamixel_t *dynamixel) {
    memset(dynamixel->registered, 0, sizeof(dynamixel->registered));
    memset(dynamixel->queue, 0, sizeof(dynamixel->queue));

    for(uint8_t i = 0; i < DYNAMIXEL_REGISTER_MAX; i++) {
        dynamixel->registered[i].id = UNREGISTRED;
    }

    dynamixel->queue_r = 0;
    dynamixel->queue_w = 0;
    dynamixel->pending = NULL;

    dynamixel->task_time = HAL_GetTick();
}

dynamixel_servo_t *dynamixel_register(dynamixel_t *dynamixel, const uint8_t id) {
    for(uint8_t i = 0; i < DYNAMIXEL_REGISTER_MAX; i++) {
        if(dynamixel->registered[i].id == UNREGISTRED) {
            dynamixel->registered[i].id = id;
            return &dynamixel->registered[i];
        }
    }
    return NULL;
}

void dynamixel_tick(dynamixel_t *dynamixel) {
    const uint32_t time = HAL_GetTick();

    if((time - dynamixel->task_time) >= 10) {
        dynamixel->task_time = time;

        uint8_t params[8];

        for(uint8_t i = 0; i < DYNAMIXEL_REGISTER_MAX; i++) {
            if(dynamixel->registered[i].id != UNREGISTRED) {
                const uint16_t goal = 614.4f * (dynamixel->registered[i].goal / PI) + 512;
                params[0] = AX12A_GOAL_POSITION_ADDRESS;
                params[1] = (goal >> 0) & 0xFF;
                params[2] = (goal >> 8) & 0xFF;
                push_instruction_packet(dynamixel, dynamixel->registered[i].id, INSTRUCTION_WRITE,
                                        params, 3, NULL);

                params[0] = AX12A_LED_ADDRESS;
                params[1] = dynamixel->registered[i].led;
                push_instruction_packet(dynamixel, dynamixel->registered[i].id, INSTRUCTION_WRITE,
                                        params, 2, NULL);

                params[0] = AX12A_PRESENT_POSITION_ADDRESS;
                params[1] = 8;
                push_instruction_packet(dynamixel, dynamixel->registered[i].id, INSTRUCTION_READ,
                                        params, 2, &dynamixel->registered[i]);
            }
        }
    }

    if(dynamixel->pending && ((time - dynamixel->start) >= 1)) {
        const uint8_t header1 = dynamixel->status_buffer[0];
        const uint8_t header2 = dynamixel->status_buffer[1];
        const uint8_t id = dynamixel->status_buffer[2];
        const uint8_t length = dynamixel->status_buffer[3];
        const uint8_t error = dynamixel->status_buffer[4];
        const uint8_t *params = &dynamixel->status_buffer[5];
        const uint8_t checksum = dynamixel->status_buffer[length + 3];

        if((header1 == 0xFF) && (header2 == 0xFF) && (length >= 2)) {
            uint8_t chksum = 0;
            chksum += id;
            chksum += length;
            chksum += error;
            for(uint8_t i = 0; i < (length - 2); i++) {
                chksum += params[i];
            }
            chksum = ~chksum;

            if(chksum == checksum) {
                if((length == 10) && dynamixel->pending->context) {
                    dynamixel_servo_t *servo = dynamixel->pending->context;

                    const float position = (uint16_t)((((uint16_t)params[1]) << 8) | params[0]);
                    const uint16_t velocity = (uint16_t)((((uint16_t)params[3]) << 8) | params[2]);
                    const uint16_t load = (uint16_t)((((uint16_t)params[5]) << 8) | params[4]);

                    servo->position = (PI / 614.4f) * (position - 512.f);
                    servo->velocity = (2.f * PI * 0.111f / 60.f) * (velocity & 0x400 ? -1.f : 1.f) *
                                      (velocity & 0x3FF);
                    servo->load = (load & 0x400 ? 1.f : -1.f) * (load & 0x3FF) / 1024.f;
                    servo->voltage = params[6] * 0.1f;
                    servo->temperature = params[7];
                    servo->error = error;
                    servo->timestamp = time;
                }
            }
        }

        dynamixel->pending = NULL;
    }

    if(!dynamixel->pending) {
        dynamixel->pending = pop_instruction_packet(dynamixel);

        if(dynamixel->pending) {
            memset(dynamixel->status_buffer, 0, sizeof(dynamixel->status_buffer));
            dynamixel->start = time;
            HAL_UART_DMAStop(dynamixel->uart);
            HAL_HalfDuplex_EnableTransmitter(dynamixel->uart);
            HAL_UART_Transmit_DMA(dynamixel->uart, (uint8_t *)dynamixel->pending->buffer,
                                  (uint16_t)dynamixel->pending->size);
        }
    }
}

void dynamixel_transmit_callback(dynamixel_t *dynamixel, const UART_HandleTypeDef *huart) {
    if(dynamixel->uart == huart) {
        HAL_HalfDuplex_EnableReceiver(dynamixel->uart);
        HAL_UART_Receive_DMA(dynamixel->uart, dynamixel->status_buffer,
                             sizeof(dynamixel->status_buffer));
    }
}
