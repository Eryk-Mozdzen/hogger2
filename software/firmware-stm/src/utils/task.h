#ifndef TASK_H
#define TASK_H

#define _TASK_REGISTER1(a, b) a##b
#define _TASK_REGISTER2(a, b) _TASK_REGISTER1(a, b)

#define TASK_REGISTER_INIT(func_)                                                                  \
    __attribute__((constructor)) static void _TASK_REGISTER2(_task_register_, __LINE__)() {        \
        static task_t task = {                                                                     \
            .func = (func_),                                                                       \
            .logic = NULL,                                                                         \
            .context = NULL,                                                                       \
        };                                                                                         \
        _task_register(task);                                                                      \
    }

#define TASK_REGISTER_NONSTOP(func_)                                                               \
    __attribute__((constructor)) static void _TASK_REGISTER2(_task_register_, __LINE__)() {        \
        static task_nonstop_t task = {                                                             \
            .base.func = (func_),                                                                  \
            .base.logic = _task_logic_nonstop,                                                     \
            .base.context = NULL,                                                                  \
        };                                                                                         \
        _task_register(task.base);                                                                 \
    }

#define TASK_REGISTER_PERIODIC(func_, period_)                                                     \
    __attribute__((constructor)) static void _TASK_REGISTER2(_task_register_, __LINE__)() {        \
        static task_periodic_t task = {                                                            \
            .base.func = (func_),                                                                  \
            .base.logic = _task_logic_periodic,                                                    \
            .base.context = &task,                                                                 \
            .period = (period_),                                                                   \
            .next = 0,                                                                             \
        };                                                                                         \
        _task_register(task.base);                                                                 \
    }

#define TASK_REGISTER_INTERRUPT(func_, flag_)                                                      \
    __attribute__((constructor)) static void _TASK_REGISTER2(_task_register_, __LINE__)() {        \
        static task_interrupt_t task = {                                                           \
            .base.func = (func_),                                                                  \
            .base.logic = _task_logic_interrupt,                                                   \
            .base.context = &task,                                                                 \
            .flag = (flag_),                                                                       \
        };                                                                                         \
        _task_register(task.base);                                                                 \
    }

typedef struct {
    void (*func)();
    uint32_t (*logic)(void *);
    void *context;
} task_t;

typedef struct {
    task_t base;
} task_nonstop_t;

typedef struct {
    task_t base;
    const uint32_t period;
    uint32_t next;
} task_periodic_t;

typedef struct {
    task_t base;
    volatile uint32_t *flag;
} task_interrupt_t;

void task_call_init();
void task_call();

void _task_register(const task_t task);
uint32_t _task_logic_nonstop(void *context);
uint32_t _task_logic_periodic(void *context);
uint32_t _task_logic_interrupt(void *context);

#endif
