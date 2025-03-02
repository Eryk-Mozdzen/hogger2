#include "com/Mailbox.hpp"
#include "com/MailboxRegistry.hpp"

MailboxBase::MailboxBase(const uint32_t minimal) : minimal{minimal} {
    MailboxRegistry &registry = MailboxRegistry::getInstance();

    next = registry.head;
    registry.head = this;
}
