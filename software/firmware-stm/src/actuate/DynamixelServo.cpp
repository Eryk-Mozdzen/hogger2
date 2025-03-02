#include "actuate/DynamixelServo.hpp"
#include "actuate/DynamixelBus.hpp"
#include "freertos/Task.hpp"

DynamixelServo::DynamixelServo(const char *name, DynamixelBus &bus, const uint8_t id)
    : id{id}, name{name} {
    bus.add(this);
}

void DynamixelServo::serialize(cmp_ctx_t *cmp) const {
    const uint32_t time = xTaskGetTickCount();

    const uint8_t valid = ((time - timestamp) <= 500);

    float pos;
    float vel;
    float ld;
    float temp;

    xSemaphoreTake(lock, portMAX_DELAY);
    pos = position;
    vel = velocity;
    ld = load;
    temp = temperature;
    xSemaphoreGive(lock);

    cmp_write_map(cmp, 5);
    cmp_write_str(cmp, "pos_ref", 7);
    cmp_write_float(cmp, goal);
    cmp_write_str(cmp, "pos", 3);
    cmp_write_float(cmp, valid ? pos : NAN);
    cmp_write_str(cmp, "vel", 3);
    cmp_write_float(cmp, valid ? vel : NAN);
    cmp_write_str(cmp, "load", 4);
    cmp_write_float(cmp, valid ? ld : NAN);
    cmp_write_str(cmp, "temp", 4);
    cmp_write_float(cmp, valid ? temp : NAN);
}

const char *DynamixelServo::getName() const {
    return name;
}

void DynamixelServo::setGoal(float position) {
    constexpr float pi = 3.141592653589f;
    constexpr float pi2 = pi / 2.f;

    if(position > pi2) {
        position = pi2;
    } else if(position < -pi2) {
        position = -pi2;
    }

    xSemaphoreTake(lock, portMAX_DELAY);
    goal = position;
    xSemaphoreGive(lock);
}

void DynamixelServo::setLed(const bool state) {
    xSemaphoreTake(lock, portMAX_DELAY);
    led = state;
    xSemaphoreGive(lock);
}
