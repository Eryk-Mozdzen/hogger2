#ifndef TELEMETRY_H
#define TELEMETRY_H

#include <cmp/cmp.h>
#include <stdint.h>

#define TELEMETRY_CONCAT_IMPL(a, b) a##b
#define TELEMETRY_CONCAT(a, b)      TELEMETRY_CONCAT_IMPL(a, b)

#define TELEMETRY_REGISTER(name, serialize, context)                                               \
    __attribute__((constructor)) static void TELEMETRY_CONCAT(telemetry_reg_, __LINE__)() {        \
        telemetry_register((name), (serialize), (context));                                        \
    }

typedef void (*telemetry_serialize_t)(cmp_ctx_t *, void *);

void telemetry_register(const char *name, const telemetry_serialize_t serialize, void *context);

#endif
