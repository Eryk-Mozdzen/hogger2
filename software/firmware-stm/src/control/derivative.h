#ifndef DERIVATIVE_H
#define DERIVATIVE_H

#include <stdint.h>

typedef struct {
    float value;
    float value_prev;
} derivative_element_t;

typedef struct {
    uint32_t time_prev;
    uint32_t dim;
    derivative_element_t *elements;
    float min;
    float max;
} derivative_t;

void derivative_init(derivative_t *derivative,
                     derivative_element_t *elements,
                     const uint32_t dim,
                     const float min,
                     const float max);
void derivative_reset(derivative_t *derivative);
void derivative_step(derivative_t *derivative, const float *input);
float derivative_get(const derivative_t *derivative, const uint32_t index);

#endif
