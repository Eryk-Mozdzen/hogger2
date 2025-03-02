#include <cstring>
#include <cstdint>

#include "com/TelemetrySerializer.hpp"
#include "com/TelemetrySource.hpp"
#include "freertos/Task.hpp"
#include "utils/cmp.hpp"

TelemetrySerializer::TelemetrySerializer() {
}

TelemetrySerializer &TelemetrySerializer::getInstance() {
    static TelemetrySerializer instance;
    return instance;
}

uint32_t TelemetrySerializer::serialize(void *buffer, const uint32_t capacity) const {
    const uint32_t timestamp = xTaskGetTickCount();

    cmp::MessagePack mpack = cmp::MessagePack::createEmpty(buffer, capacity);

    cmp_write_map(&mpack.ctx, count + 1);

    cmp_write_str(&mpack.ctx, "timestamp", 9);
    cmp_write_u32(&mpack.ctx, timestamp);

    TelemetrySource *current = head;

    while(current != nullptr) {
        const char *name = current->getName();
        cmp_write_str(&mpack.ctx, name, strlen(name));

        current->serialize(&mpack.ctx);

        current = current->next;
    }

    return mpack.size;
}
