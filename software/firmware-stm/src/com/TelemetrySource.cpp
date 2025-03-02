#include "com/TelemetrySource.hpp"
#include "com/TelemetryRegistry.hpp"

TelemetrySource::TelemetrySource() {
    TelemetryRegistry::getInstance().add(this);
}
