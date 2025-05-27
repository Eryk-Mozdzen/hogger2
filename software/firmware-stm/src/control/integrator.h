#ifndef INTEGRATOR_H
#define INTEGRATOR_H

#include <stdint.h>

typedef struct {
    float value;
    float value_prev;
} integrator_element_t;

typedef struct {
    uint32_t time_prev;
    uint32_t dim;
    integrator_element_t *elements;
    float min;
    float max;
} integrator_t;

void integrator_init(integrator_t *integrator,
                     integrator_element_t *elements,
                     const uint32_t dim,
                     const float min,
                     const float max);
void integrator_reset(integrator_t *integrator);
void integrator_step(integrator_t *integrator, const float *input);
void integrator_set(integrator_t *integrator, const uint32_t index, const float value);
float integrator_get(const integrator_t *integrator, const uint32_t index);

#endif
