#include <cstring>
#include <stm32u5xx_hal.h>

#include "actuate/DynamixelAX12A.h"
#include "actuate/DynamixelBus.hpp"
#include "actuate/DynamixelServo.hpp"
#include "freertos/Task.hpp"

DynamixelBus::DynamixelBus(UART_HandleTypeDef *uart) : uart{uart} {
}

void DynamixelBus::add(DynamixelServo *servo) {
    servo->next = head;
    head = servo;
}

uint8_t DynamixelBus::transaction(DynamixelServo *servo,
                                  const DynamixelBus::Instruction instruction,
                                  const uint8_t paramsSize) {
    const uint8_t length = paramsSize + 2;
    const uint8_t size = paramsSize + 6;

    packet[0] = 0xFF;
    packet[1] = 0xFF;
    packet[2] = servo->id;
    packet[3] = length;
    packet[4] = instruction;

    uint8_t checksum = 0;
    checksum += servo->id;
    checksum += length;
    checksum += instruction;

    for(uint8_t i = 0; i < paramsSize; i++) {
        packet[i + 5] = params[i];
        checksum += params[i];
    }

    packet[paramsSize + 5] = ~checksum;

    HAL_UART_DMAStop(uart);
    HAL_HalfDuplex_EnableTransmitter(uart);
    HAL_UART_Transmit_DMA(uart, packet, size);

    vTaskDelay(1);

    const uint32_t resp_size = sizeof(packet) - __HAL_DMA_GET_COUNTER(uart->hdmarx);

    if(resp_size >= 6) {
        const uint8_t resp_id = packet[2];
        const uint8_t resp_length = packet[3];
        const uint8_t resp_error = packet[4];
        const uint8_t *resp_params = &packet[5];
        const uint8_t resp_checksum = packet[resp_length + 3];

        if((packet[0] == 0xFF) && (packet[1] == 0xFF) && (resp_id == servo->id) &&
           (resp_length >= 2)) {
            uint8_t chksum = 0;
            chksum += resp_id;
            chksum += resp_length;
            chksum += resp_error;
            for(uint8_t i = 0; i < (resp_length - 2); i++) {
                chksum += resp_params[i];
            }
            chksum = ~chksum;

            if(chksum == resp_checksum) {
                memcpy(params, resp_params, resp_length - 2);
                return (resp_length - 2);
            }
        }
    }

    return 0;
}

void DynamixelBus::tick() {
    constexpr float pi = 3.1415f;

    DynamixelServo *current = head;

    while(current != nullptr) {
        params[0] = DYNAMIXEL_AX12A_GOAL_POSITION_ADDRESS;
        xSemaphoreTake(current->lock, portMAX_DELAY);
        const uint16_t goal = 614.4f * (current->goal / pi) + 512;
        params[1] = (goal >> 0) & 0xFF;
        params[2] = (goal >> 8) & 0xFF;
        xSemaphoreGive(current->lock);
        transaction(current, WRITE, 3);

        params[0] = DYNAMIXEL_AX12A_LED_ADDRESS;
        xSemaphoreTake(current->lock, portMAX_DELAY);
        params[1] = current->led;
        xSemaphoreGive(current->lock);
        transaction(current, WRITE, 2);

        params[0] = DYNAMIXEL_AX12A_PRESENT_POSITION_ADDRESS;
        params[1] = 8;
        if(transaction(current, READ, 2) == 8) {
            const float position = (uint16_t) ((((uint16_t) params[1]) << 8) | params[0]);
            const uint16_t velocity = (uint16_t) ((((uint16_t) params[3]) << 8) | params[2]);
            const uint16_t load = (uint16_t) ((((uint16_t) params[5]) << 8) | params[4]);

            xSemaphoreTake(current->lock, portMAX_DELAY);
            current->position = (pi / 614.4f) * (position - 512.f);
            current->velocity =
                (2.f * pi * 0.111f / 60.f) * ((velocity & 0x400) ? -1.f : 1.f) * (velocity & 0x3FF);
            current->load = ((load & 0x400) ? 1.f : -1.f) * (load & 0x3FF) / 1024.f;
            current->temperature = params[7];
            current->timestamp = xTaskGetTickCount();
            xSemaphoreGive(current->lock);
        }

        current = current->next;
    }
}

void DynamixelBus::transmitISR(const UART_HandleTypeDef *huart) {
    if(uart == huart) {
        HAL_HalfDuplex_EnableReceiver(uart);
        HAL_UART_Receive_DMA(uart, packet, sizeof(packet));
    }
}
