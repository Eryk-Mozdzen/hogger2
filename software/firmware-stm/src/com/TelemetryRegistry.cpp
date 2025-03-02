#include <cstring>

#include "com/TelemetryRegistry.hpp"
#include "com/TelemetrySource.hpp"
#include "com/cmp.hpp"
#include "freertos/Task.hpp"

TelemetryRegistry &TelemetryRegistry::getInstance() {
    static TelemetryRegistry instance;
    return instance;
}

void TelemetryRegistry::add(TelemetrySource *source) {
    source->next = head;
    head = source;
    count++;
}

void TelemetryRegistry::serialize(cmp::MessagePack &mpack) const {
    const uint32_t timestamp = xTaskGetTickCount();

    cmp_ctx_t cmp;
    cmp_init(&cmp, &mpack, NULL, NULL, cmp::writer);

    cmp_write_map(&cmp, count + 1);

    cmp_write_str(&cmp, "timestamp", 9);
    cmp_write_u32(&cmp, timestamp);

    TelemetrySource *current = head;

    while(current != nullptr) {
        const char *name = current->getName();
        cmp_write_str(&cmp, name, strlen(name));

        current->serialize(&cmp);

        current = current->next;
    }
}
