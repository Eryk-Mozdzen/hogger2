#pragma once

#include <FreeRTOS.h>
#include <semphr.h>

namespace freertos {

class Mutex {
    StaticSemaphore_t buffer;
    SemaphoreHandle_t handle;

public:
    Mutex() {
        handle = xSemaphoreCreateMutexStatic(&buffer);
    }

    operator SemaphoreHandle_t() const {
        return handle;
    }
};

}
