#pragma once

#include <cmp/cmp.h>

namespace cmp {

class MessagePack {
    uint32_t capacity;
    uint32_t position;

    MessagePack() = default;

    static size_t writer(cmp_ctx_t *ctx, const void *data, size_t count);
    static bool reader(cmp_ctx_t *ctx, void *data, size_t count);

public:
    cmp_ctx_t ctx;
    uint8_t *buffer;
    uint32_t size;

    static MessagePack createEmpty(void *buffer, const uint32_t capacity);
    static MessagePack createFromData(const void *buffer, const uint32_t size);
};

}
