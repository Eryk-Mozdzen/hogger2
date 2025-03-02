#pragma once

#include "com/TelemetrySource.hpp"
#include "freertos/Mutex.hpp"

class DynamixelBus;

class DynamixelServo : TelemetrySource {
    friend class DynamixelBus;

    const uint8_t id;
    const char *name;

    freertos::Mutex lock;

    float goal;
    bool led;

    float position;
    float velocity;
    float load;
    float temperature;
    uint32_t timestamp;

    DynamixelServo *next = nullptr;

    void serialize(cmp_ctx_t *cmp) const;
    const char *getName() const;

public:
    DynamixelServo(const char *name, DynamixelBus &bus, const uint8_t id);

    void setGoal(float position);
    void setLed(const bool state);
};
