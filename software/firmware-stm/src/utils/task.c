#include <stdint.h>
#include <stm32h5xx_hal.h>

#include "com/telemetry.h"
#include "utils/task.h"

#define MAX 32

extern TIM_HandleTypeDef htim12;
extern TIM_HandleTypeDef htim15;

static task_t registered[MAX] = {0};
static uint32_t count = 0;
static uint32_t computation = 0;

void _task_register(const task_t task) {
    registered[count] = task;
    count++;
}

void task_call_init() {
    for(uint32_t i = 0; i < count; i++) {
        if(!registered[i].logic) {
            registered[i].func();
        }
    }

    HAL_TIM_Base_Start(&htim12);
    HAL_TIM_Base_Start(&htim15);
}

void task_call() {
    for(uint32_t i = 0; i < count; i++) {
        if(registered[i].logic) {
            if(registered[i].logic(registered[i].context)) {
                const uint32_t start = task_timebase();
                registered[i].func();
                computation += (task_timebase() - start);
            }
        }
    }
}

uint32_t task_timebase() {
    const uint32_t slave = __HAL_TIM_GET_COUNTER(&htim15);
    const uint32_t master = __HAL_TIM_GET_COUNTER(&htim12);
    return ((slave << 16) | master);
}

uint32_t _task_logic_periodic(void *context) {
    task_periodic_t *task = context;
    if(task_timebase() > task->next) {
        task->next += task->period;
        return 1;
    }
    return 0;
}

uint32_t _task_logic_interrupt(void *context) {
    task_interrupt_t *task = context;
    if(*task->flag) {
        *task->flag = 0;
        return 1;
    }
    return 0;
}

static void serialize(cmp_ctx_t *cmp, void *context) {
    (void)context;

    const float load = (100.f * computation) / 20000;

    computation = 0;

    cmp_write_u8(cmp, load);
}

TELEMETRY_REGISTER("core_load", serialize, NULL)
