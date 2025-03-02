#pragma once

#include <cmp/cmp.h>

class TelemetrySerializer;

class TelemetrySource {
    friend class TelemetrySerializer;

    TelemetrySource *next = nullptr;

    virtual void serialize(cmp_ctx_t *cmp) const = 0;
    virtual const char *getName() const = 0;

protected:
    TelemetrySource();
};
