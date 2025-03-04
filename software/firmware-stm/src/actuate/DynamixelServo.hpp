#pragma once

#include "com/TelemetrySource.hpp"
#include "freertos/Mutex.hpp"

class DynamixelBus;

class DynamixelServo : TelemetrySource {
    friend class DynamixelBus;

    const uint8_t id;
    const char *name;

    freertos::Mutex lock;

    float goal = 0;
    bool led = false;

    float position = 0;
    float velocity = 0;
    float load = 0;
    float temperature = 0;
    uint32_t timestamp = 0;

    DynamixelServo *next = nullptr;

    void serialize(cmp_ctx_t *cmp) const;
    const char *getName() const;

public:
    DynamixelServo(const char *name, DynamixelBus &bus, const uint8_t id);

    void setGoal(float position);
    void setLed(const bool state);
};
