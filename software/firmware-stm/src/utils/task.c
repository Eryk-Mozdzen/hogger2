#include <stdint.h>
#include <stm32u5xx_hal.h>

#include "utils/task.h"

#define MAX 16

typedef struct {
    task_t task;
    task_type_t type;
    uint32_t period;
    uint32_t next;
} registered_t;

static registered_t registered[MAX] = {0};
static uint32_t count = 0;

void task_register(const task_t task, const task_type_t type, const uint32_t period) {
    registered[count].task = task;
    registered[count].type = type;
    registered[count].period = period;
    registered[count].next = 0;
    count++;
}

void task_call_init() {
    for(uint32_t i = 0; i < count; i++) {
        if((registered[i].type == TASK_INIT) && registered[i].task) {
            registered[i].task();
        }
    }
}

void task_call_periodic() {
    const uint32_t time = HAL_GetTick();

    for(uint32_t i = 0; i < count; i++) {
        if((registered[i].type == TASK_PERIODIC) && (registered[i].next < time) &&
           registered[i].task) {
            registered[i].task();
            registered[i].next += registered[i].period;
        }
    }
}
