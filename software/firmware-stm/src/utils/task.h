#ifndef TASK_H
#define TASK_H

#define _TASK_REGISTER_IMPL(a, b) a##b
#define _TASK_REGISTER(a, b)      _TASK_REGISTER_IMPL(a, b)

#define TASK_REGISTER_INIT(task)                                                                   \
    __attribute__((constructor)) static void _TASK_REGISTER(_task_register_, __LINE__)() {         \
        task_register((task), NULL);                                                               \
    }

#define TASK_REGISTER_NONSTOP(task)                                                                \
    __attribute__((constructor)) static void _TASK_REGISTER(_task_register_, __LINE__)() {         \
        static task_trigger_nonstop_t trigger = {                                                  \
            .base.logic = _task_trigger_logic_nonstop,                                             \
            .base.context = NULL,                                                                  \
        };                                                                                         \
        task_register((task), &trigger.base);                                                      \
    }

#define TASK_REGISTER_PERIODIC(task, period_)                                                      \
    __attribute__((constructor)) static void _TASK_REGISTER(_task_register_, __LINE__)() {         \
        static task_trigger_periodic_t trigger = {                                                 \
            .base.logic = _task_trigger_logic_periodic,                                            \
            .base.context = &trigger,                                                              \
            .period = (period_),                                                                   \
            .next = 0,                                                                             \
        };                                                                                         \
        task_register((task), &trigger.base);                                                      \
    }

#define TASK_REGISTER_INTERRUPT(task, flag_)                                                       \
    __attribute__((constructor)) static void _TASK_REGISTER(_task_register_, __LINE__)() {         \
        static task_trigger_interrupt_t trigger = {                                                \
            .base.logic = _task_trigger_logic_interrupt,                                           \
            .base.context = &trigger,                                                              \
            .flag = (flag_),                                                                       \
        };                                                                                         \
        task_register((task), &trigger.base);                                                      \
    }

typedef struct {
    uint32_t (*logic)(void *);
    void *context;
} task_trigger_t;

typedef struct {
    task_trigger_t base;
} task_trigger_nonstop_t;

typedef struct {
    task_trigger_t base;
    const uint32_t period;
    uint32_t next;
} task_trigger_periodic_t;

typedef struct {
    task_trigger_t base;
    volatile uint32_t *flag;
} task_trigger_interrupt_t;

typedef void (*task_t)();

void task_register(const task_t task, task_trigger_t *trigger);
void task_call_init();
void task_call();

uint32_t _task_trigger_logic_nonstop(void *context);
uint32_t _task_trigger_logic_periodic(void *context);
uint32_t _task_trigger_logic_interrupt(void *context);

#endif
