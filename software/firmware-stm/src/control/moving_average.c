#include "control/moving_average.h"

void moving_average_reset(moving_average_t *instance) {
    for(uint32_t i = 0; i < instance->length; i++) {
        instance->buffer[i] = 0;
    }

    instance->sum = 0;
    instance->counter = 0;
}

float moving_average_append(moving_average_t *instance, const float input) {
    instance->sum -= instance->buffer[instance->counter];
    instance->buffer[instance->counter] = input;
    instance->sum += instance->buffer[instance->counter];
    instance->counter++;
    instance->counter %= instance->length;

    return (instance->sum / ((float)instance->length));
}
