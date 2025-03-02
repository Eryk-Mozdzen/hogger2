#pragma once

#include <FreeRTOS.h>
#include <task.h>

namespace freertos {

class TaskBase {
protected:
    TaskHandle_t handle;

public:
    operator TaskHandle_t() const {
        return handle;
    }
};

template <uint32_t STACK>
class TaskClass : public TaskBase {
    StaticTask_t tcb;
    StackType_t stack[STACK];

    virtual void task() = 0;

    static void caller(void *params) {
        TaskClass *object = static_cast<TaskClass *>(params);
        (object->task)();
    }

public:
    TaskClass(const char *name, const UBaseType_t priority) {
        handle = xTaskCreateStatic(&TaskClass::caller, name, STACK, this, priority, stack, &tcb);
    }
};

template <class T, uint32_t STACK>
class TaskMember : public TaskBase {
    StaticTask_t tcb;
    StackType_t stack[STACK];

    T *object;
    void (T::*task)();

    static void caller(void *params) {
        TaskMember *member = static_cast<TaskMember *>(params);
        (member->object->*member->task)();
    }

public:
    TaskMember(const char *name, T *object, void (T::*task)(), const UBaseType_t priority)
        : object{object}, task{task} {
        handle = xTaskCreateStatic(&TaskMember::caller, name, STACK, this, priority, stack, &tcb);
    }
};

}
