#include <cmp/cmp.h>
#include <cstring>

#include "utils/cmp.hpp"

namespace cmp {

MessagePack MessagePack::createEmpty(void *buffer, const uint32_t capacity) {
    MessagePack mpack;
    mpack.buffer = static_cast<uint8_t *>(buffer);
    mpack.capacity = capacity;
    mpack.size = 0;
    mpack.position = 0;

    cmp_init(&mpack.ctx, &mpack, MessagePack::reader, NULL, MessagePack::writer);

    return mpack;
}

MessagePack MessagePack::createFromData(const void *buffer, const uint32_t size) {
    MessagePack mpack;
    mpack.buffer = (uint8_t *) buffer;
    mpack.capacity = size;
    mpack.size = size;
    mpack.position = 0;

    cmp_init(&mpack.ctx, &mpack, MessagePack::reader, NULL, MessagePack::writer);

    return mpack;
}

size_t MessagePack::writer(cmp_ctx_t *ctx, const void *data, size_t count) {
    MessagePack *buf = static_cast<MessagePack *>(ctx->buf);

    if((buf->position + count) > buf->capacity) {
        return 0;
    }

    memcpy(buf->buffer + buf->position, data, count);
    buf->position += count;
    buf->size = buf->position;
    return count;
}

bool MessagePack::reader(cmp_ctx_t *ctx, void *data, size_t count) {
    MessagePack *buf = static_cast<MessagePack *>(ctx->buf);

    if((buf->position + count) > buf->size) {
        return false;
    }

    memcpy(data, buf->buffer + buf->position, count);
    buf->position += count;
    return true;
}

}
