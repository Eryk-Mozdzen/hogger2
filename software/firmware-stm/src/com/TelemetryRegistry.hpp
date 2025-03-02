#pragma once

#include "com/cmp.hpp"

class TelemetrySource;

class TelemetryRegistry {
    friend class TelemetrySource;

    TelemetrySource *head = nullptr;
    uint32_t count = 0;

    TelemetryRegistry() = default;

    void add(TelemetrySource *source);

public:
    static TelemetryRegistry &getInstance();

    void serialize(cmp::MessagePack &mpack) const;
};
