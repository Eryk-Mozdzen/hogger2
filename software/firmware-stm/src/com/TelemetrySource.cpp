#include "com/TelemetrySource.hpp"
#include "com/TelemetrySerializer.hpp"

TelemetrySource::TelemetrySource() {
    TelemetrySerializer &serializer = TelemetrySerializer::getInstance();

    next = serializer.head;
    serializer.head = this;
    serializer.count++;
}
