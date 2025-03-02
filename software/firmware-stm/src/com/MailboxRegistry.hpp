#pragma once

#include "freertos/MessageBuffer.hpp"

class MailboxBase;

class MailboxRegistry {
    friend class MailboxBase;

    MailboxBase *head = nullptr;

    MailboxRegistry();

public:
    static MailboxRegistry &getInstance();

    void send(const void *data, const uint32_t size);
};
