#ifndef CONFIG_H
#define CONFIG_H

#include <stdint.h>

#define CONFIG_CONCAT_IMPL(a, b) a##b
#define CONFIG_CONCAT(a, b)      CONFIG_CONCAT_IMPL(a, b)

#define CONFIG_REGISTER(name, vector, dim)                                                         \
    __attribute__((constructor)) static void CONFIG_CONCAT(config_reg_, __LINE__)() {              \
        config_register((name), (vector), (dim));                                                  \
    }

void config_register(const char *name, float *vector, const uint32_t dim);

#endif
