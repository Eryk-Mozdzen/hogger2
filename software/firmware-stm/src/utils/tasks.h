#ifndef TASKS_H
#define TASKS_H

#define TASKS_CONCAT_IMPL(a, b) a##b
#define TASKS_CONCAT(a, b)      TASKS_CONCAT_IMPL(a, b)

#define TASKS_REGISTER(init, loop, period)                                                         \
    __attribute__((constructor)) static void TASKS_CONCAT(tasks_reg_, __LINE__)() {                \
        tasks_register((init), (loop), (period));                                                  \
    }

typedef void (*task_t)();

void tasks_register(const task_t init, const task_t loop, const uint32_t period);
void tasks_init();
void tasks_loop();

#endif
