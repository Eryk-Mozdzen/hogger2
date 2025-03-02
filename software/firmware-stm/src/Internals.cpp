#include "freertos/MessageBuffer.hpp"

namespace internals {

freertos::MessageBufferBase &comTxQueue() {
    static freertos::MessageBuffer<10 * 1024> buffer;
    return buffer;
}

freertos::MessageBufferBase &comRxQueue() {
    static freertos::MessageBuffer<10 * 1024> buffer;
    return buffer;
}

}
