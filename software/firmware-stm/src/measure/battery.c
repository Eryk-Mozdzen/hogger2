#include <stm32u5xx_hal.h>

#include "com/telemetry.h"
#include "utils/task.h"

extern ADC_HandleTypeDef hadc1;

static float voltage;

static void sample() {
    const uint32_t supply_raw = HAL_ADC_GetValue(&hadc1);

    voltage = (6.1f * 3.3f * supply_raw) / ((float)(1 << 14));

    HAL_ADC_Start(&hadc1);
}

static void serialize(cmp_ctx_t *cmp, void *context) {
    (void)context;

    cmp_write_float(cmp, voltage);
}

TASK_REGISTER_PERIODIC(sample, 50000)
TELEMETRY_REGISTER("battery", serialize, NULL)
