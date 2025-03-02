#pragma once

#include <FreeRTOS.h>
#include <timers.h>

namespace freertos {

template <class T>
class TimerMember {
    StaticTimer_t timerBuffer;
    TimerHandle_t handle;

    T *object;
    void (T::*function)();

    static void caller(TimerHandle_t handle) {
        TimerMember *member = static_cast<TimerMember *>(pvTimerGetTimerID(handle));
        (member->object->*member->function)();
    }

public:
    TimerMember(const char *name,
                T *object,
                void (T::*function)(),
                const TickType_t period,
                const UBaseType_t reload)
        : object{object}, function{function} {
        handle = xTimerCreateStatic(name, period, reload, this, &TimerMember::caller, &timerBuffer);
    }

    operator TimerHandle_t() const {
        return handle;
    }
};

}
