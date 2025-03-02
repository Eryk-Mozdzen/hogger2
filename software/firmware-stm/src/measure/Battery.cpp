#include <stm32u5xx_hal.h>

#include "com/TelemetrySource.hpp"
#include "freertos/Mutex.hpp"
#include "freertos/Task.hpp"

extern ADC_HandleTypeDef hadc1;

class Battery : freertos::TaskClass<128>, TelemetrySource {
    freertos::Mutex lock;
    float voltage;

    void task();
    void serialize(cmp_ctx_t *cmp) const;
    const char *getName() const;

public:
    Battery();
};

Battery battery;

Battery::Battery() : TaskClass{"battery", 2}, voltage{0} {
}

void Battery::task() {
    uint32_t time = 0;

    while(true) {
        HAL_ADC_Start(&hadc1);
        HAL_ADC_PollForConversion(&hadc1, 2);
        const uint32_t meas = HAL_ADC_GetValue(&hadc1);

        xSemaphoreTake(lock, portMAX_DELAY);
        voltage = (6.1f * 3.3f * meas) / ((float) (1 << 14));
        xSemaphoreGive(lock);

        vTaskDelayUntil(&time, 50);
    }
}

void Battery::serialize(cmp_ctx_t *cmp) const {
    xSemaphoreTake(lock, portMAX_DELAY);
    const float volt = voltage;
    xSemaphoreGive(lock);

    cmp_write_float(cmp, volt);
}

const char *Battery::getName() const {
    return "battery";
}
