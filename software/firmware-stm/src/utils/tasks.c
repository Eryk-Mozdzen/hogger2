#include <stdint.h>
#include <stm32u5xx_hal.h>

#include "tasks.h"

#define MAX 16

typedef struct {
    task_t init;
    task_t loop;
    uint32_t period;
    uint32_t next;
} registered_t;

static registered_t registered[MAX] = {0};
static uint32_t count = 0;

void tasks_register(const task_t init, const task_t loop, const uint32_t period) {
    registered[count].init = init;
    registered[count].loop = loop;
    registered[count].period = period;
    registered[count].next = 0;
    count++;
}

void tasks_init() {
    for(uint32_t i = 0; i < count; i++) {
        if(registered[i].init) {
            registered[i].init();
        }
    }
}

void tasks_loop() {
    const uint32_t time = HAL_GetTick();

    for(uint32_t i = 0; i < count; i++) {
        if(registered[i].loop && (registered[i].next < time)) {
            registered[i].loop();
            registered[i].next += registered[i].period;
        }
    }
}
