#ifndef CONFIG_H
#define CONFIG_H

#include <stdint.h>

#define _CONFIG_REGISTER1(a, b) a##b
#define _CONFIG_REGISTER2(a, b) _CONFIG_REGISTER1(a, b)

#define CONFIG_REGISTER(name, vector, dim)                                                         \
    __attribute__((constructor)) static void _CONFIG_REGISTER2(config_reg_, __LINE__)() {          \
        config_register((name), (vector), (dim));                                                  \
    }

void config_register(const char *name, float *vector, const uint32_t dim);

#endif
