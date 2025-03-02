#pragma once

#include <FreeRTOS.h>
#include <message_buffer.h>

namespace freertos {

class MessageBufferBase {
protected:
    MessageBufferHandle_t handle;

public:
    operator MessageBufferHandle_t() const {
        return handle;
    }
};

template <uint32_t SIZE>
class MessageBuffer : public MessageBufferBase {
    StaticMessageBuffer_t msgBuffer;
    uint8_t storageBuffer[SIZE];

public:
    MessageBuffer() {
        handle = xMessageBufferCreateStatic(SIZE, storageBuffer, &msgBuffer);
    }
};

}
