#pragma once

#include <cmp/cmp.h>

namespace cmp {

class MessagePack {
    MessagePack() = default;

public:
    uint8_t *buffer;
    uint32_t capacity;
    uint32_t size;
    uint32_t position;

    static MessagePack createEmpty(void *buffer, const uint32_t capacity);
    static MessagePack createFromData(const void *buffer, const uint32_t size);
};

size_t writer(cmp_ctx_t *ctx, const void *data, size_t count);
bool reader(cmp_ctx_t *ctx, void *data, size_t count);

}
