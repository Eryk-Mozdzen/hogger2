#ifndef INTERRUPT_H
#define INTERRUPT_H

#include <stdint.h>

typedef void (*interrupt_callback_t)();

void interrupt_register(const interrupt_callback_t callback, const uint16_t pin);

#endif
