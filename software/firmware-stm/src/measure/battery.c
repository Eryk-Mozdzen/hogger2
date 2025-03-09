#include <stm32u5xx_hal.h>

#include "com/telemetry.h"

extern ADC_HandleTypeDef hadc1;

static void serialize(cmp_ctx_t *cmp, void *context) {
    (void)context;

    HAL_ADC_Start(&hadc1);
    HAL_ADC_PollForConversion(&hadc1, 2);
    const uint32_t supply_raw = HAL_ADC_GetValue(&hadc1);
    const float supply = (6.1f * 3.3f * supply_raw) / ((float)(1 << 14));

    cmp_write_float(cmp, supply);
}

TELEMETRY_REGISTER("battery", serialize, NULL)
