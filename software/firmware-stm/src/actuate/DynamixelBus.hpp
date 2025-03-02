#pragma once

#include <stm32u5xx_hal.h>

#include "freertos/Task.hpp"

class DynamixelServo;

class DynamixelBus {
    friend class DynamixelServo;

    enum Instruction {
        PING = 0x01,
        READ = 0x02,
        WRITE = 0x03,
        REG_WRITE = 0x04,
        ACTION = 0x05,
        FACTORY_RESET = 0x06,
        REBOOT = 0x08,
        SYNC_WRITE = 0x83,
        BULK_READ = 0x92,
    };

    uint8_t packet[300];
    uint8_t params[300];

    DynamixelServo *head = nullptr;
    UART_HandleTypeDef *uart;

    void add(DynamixelServo *servo);
    uint8_t transaction(DynamixelServo *servo,
                        const DynamixelBus::Instruction instruction,
                        const uint8_t paramsSize);

public:
    DynamixelBus(UART_HandleTypeDef *uart);

    void tick();

    void transmitISR(const UART_HandleTypeDef *huart);
};
