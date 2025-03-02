#pragma once

#include <cstdint>

class TelemetrySource;

class TelemetrySerializer {
    friend class TelemetrySource;

    TelemetrySource *head = nullptr;
    uint32_t count = 0;

    TelemetrySerializer();

public:
    static TelemetrySerializer &getInstance();

    uint32_t serialize(void *buffer, const uint32_t capacity) const;
};
