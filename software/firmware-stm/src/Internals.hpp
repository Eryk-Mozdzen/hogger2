#pragma once

#include "freertos/MessageBuffer.hpp"

namespace internals {

freertos::MessageBufferBase &comTxQueue();
freertos::MessageBufferBase &comRxQueue();

}
