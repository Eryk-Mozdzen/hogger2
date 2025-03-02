#include "com/MailboxRegistry.hpp"
#include "com/Mailbox.hpp"
#include "freertos/MessageBuffer.hpp"

MailboxRegistry::MailboxRegistry() {
}

MailboxRegistry &MailboxRegistry::getInstance() {
    static MailboxRegistry instance;
    return instance;
}

void MailboxRegistry::send(const void *data, const uint32_t size) {
    MailboxBase *current = head;

    while(current != nullptr) {
        if(size >= current->minimal) {
            xMessageBufferSend(current->getBuffer(), data, size, 0);
        }

        current = current->next;
    }
}
