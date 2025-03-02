#pragma once

#include "freertos/MessageBuffer.hpp"

class MailboxRegistry;

class MailboxBase {
    friend class MailboxRegistry;

    const uint32_t minimal;
    MailboxBase *next = nullptr;

protected:
    MailboxBase(const uint32_t minimal);

    virtual freertos::MessageBufferBase &getBuffer() = 0;
};

template <uint32_t SIZE>
class Mailbox : MailboxBase {
    freertos::MessageBuffer<SIZE> mailbox;

public:
    Mailbox(const uint32_t minimal) : MailboxBase{minimal} {
    }

    freertos::MessageBufferBase &getBuffer() {
        return mailbox;
    }
};
