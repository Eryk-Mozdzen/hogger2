#ifndef WATCHDOG_H
#define WATCHDOG_H

#define _WATCHDOG_REGISTER1(a, b) a##b
#define _WATCHDOG_REGISTER2(a, b) _WATCHDOG_REGISTER1(a, b)

#define WATCHDOG_REGISTER(callback)                                                                \
    __attribute__((constructor)) static void _WATCHDOG_REGISTER2(telemetry_reg_, __LINE__)() {     \
        watchdog_register((callback));                                                             \
    }

typedef void (*watchdog_callback_t)();

void watchdog_register(const watchdog_callback_t callback);
void watchdog_reset();

#endif
