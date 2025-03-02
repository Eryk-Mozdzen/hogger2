#include <lrcp/frame.h>
#include <stm32u5xx_hal.h>

#include "Internals.hpp"
#include "cmp.hpp"
#include "com/TelemetryRegistry.hpp"
#include "freertos/Task.hpp"
#include "lrcp.hpp"

class Telemetry : public freertos::TaskClass<1024> {
    uint8_t memory[1024];
    uint8_t encoded[1024];

public:
    Telemetry();
    void task();
};

Telemetry telemetry;

Telemetry::Telemetry() : TaskClass{"telemetry", 3} {
}

void Telemetry::task() {

    uint32_t time = 0;

    while(true) {
        cmp::MessagePack mpack = cmp::MessagePack::createEmpty(memory, sizeof(memory));

        TelemetryRegistry::getInstance().serialize(mpack);

        lrcp::Stream stream(encoded, sizeof(encoded));
        lrcp_frame_encode(&stream, mpack.buffer, mpack.size);

        xMessageBufferSend(internals::comTxQueue(), stream.buffer, stream.wr, portMAX_DELAY);

        vTaskDelayUntil(&time, 20);
    }
}
