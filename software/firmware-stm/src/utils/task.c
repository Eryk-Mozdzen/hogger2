#include <stdint.h>
#include <stm32u5xx_hal.h>

#include "utils/task.h"

#define MAX 32

static task_t registered[MAX] = {0};
static uint32_t count = 0;

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
}

void task_call() {
    for(uint32_t i = 0; i < count; i++) {
        if(registered[i].logic) {
            if(registered[i].logic(registered[i].context)) {
                registered[i].func();
            }
        }
    }
}

uint32_t _task_logic_nonstop(void *context) {
    (void)context;
    return 1;
}

uint32_t _task_logic_periodic(void *context) {
    task_periodic_t *task = context;
    if(uwTick > task->next) {
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
