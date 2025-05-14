#ifndef MOVING_AVERAGE_H
#define MOVING_AVERAGE_H

#include <stdint.h>

#define MOVING_AVERAGE_INIT(_buffer, _length)                                                      \
    {                                                                                              \
        .buffer = (_buffer),                                                                       \
        .length = (_length),                                                                       \
    }

typedef struct {
    float *const buffer;
    const uint32_t length;

    float sum;
    uint32_t counter;
} moving_average_t;

void moving_average_reset(moving_average_t *average);
float moving_average_append(moving_average_t *average, const float input);

#endif
