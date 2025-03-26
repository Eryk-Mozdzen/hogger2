#include <stm32h5xx_hal.h>

#include "com/telemetry.h"
#include "utils/task.h"

#define R1  47000.f
#define R2  10000.f
#define VCC 3.3f

extern ADC_HandleTypeDef hadc1;

static float voltage;

static void sample() {
    const uint32_t supply_raw = HAL_ADC_GetValue(&hadc1);

    voltage = (((R1 + R2) / R2) * VCC) * ((float)supply_raw) / ((float)(1 << 12));

    HAL_ADC_Start(&hadc1);
}

static void serialize(cmp_ctx_t *cmp, void *context) {
    (void)context;

    cmp_write_float(cmp, voltage);
}

TASK_REGISTER_PERIODIC(sample, 50000)
TELEMETRY_REGISTER("battery", serialize, NULL)
