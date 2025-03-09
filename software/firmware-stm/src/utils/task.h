#ifndef TASK_H
#define TASK_H

#define _TASK_REGISTER_IMPL(a, b) a##b
#define _TASK_REGISTER(a, b)      _TASK_REGISTER_IMPL(a, b)

#define TASK_REGISTER_INIT(task)                                                                   \
    __attribute__((constructor)) static void _TASK_REGISTER(_task_register_, __LINE__)() {         \
        task_register((task), TASK_INIT, 0);                                                       \
    }

#define TASK_REGISTER_PERIODIC(task, period)                                                       \
    __attribute__((constructor)) static void _TASK_REGISTER(_task_register_, __LINE__)() {         \
        task_register((task), TASK_PERIODIC, (period));                                            \
    }

typedef enum {
    TASK_INIT,
    TASK_PERIODIC,
} task_type_t;

typedef void (*task_t)();

void task_register(const task_t task, const task_type_t type, const uint32_t period);
void task_call_init();
void task_call_periodic();

#endif
