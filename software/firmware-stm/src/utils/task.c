#include <stdint.h>
#include <stm32u5xx_hal.h>

#include "utils/task.h"

#define MAX 32

typedef struct {
    task_t task;
    task_trigger_t *trigger;
} registered_t;

static registered_t registered[MAX] = {0};
static uint32_t count = 0;

void task_register(const task_t task, task_trigger_t *trigger) {
    registered[count].task = task;
    registered[count].trigger = trigger;
    count++;
}

void task_call_init() {
    for(uint32_t i = 0; i < count; i++) {
        if(!registered[i].trigger) {
            registered[i].task();
        }
    }
}

void task_call() {
    for(uint32_t i = 0; i < count; i++) {
        if(registered[i].trigger) {
            if(registered[i].trigger->logic(registered[i].trigger->context)) {
                registered[i].task();
            }
        }
    }
}

uint32_t _task_trigger_logic_nonstop(void *context) {
    (void)context;
    return 1;
}

uint32_t _task_trigger_logic_periodic(void *context) {
    task_trigger_periodic_t *trigger = context;
    if(uwTick > trigger->next) {
        trigger->next += trigger->period;
        return 1;
    }
    return 0;
}

uint32_t _task_trigger_logic_interrupt(void *context) {
    task_trigger_interrupt_t *trigger = context;
    if(*trigger->flag) {
        *trigger->flag = 0;
        return 1;
    }
    return 0;
}
