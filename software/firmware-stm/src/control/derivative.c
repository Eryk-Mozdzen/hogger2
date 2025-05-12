#include "control/derivative.h"
#include "utils/task.h"

void derivative_init(derivative_t *derivative,
                     derivative_element_t *elements,
                     const uint32_t dim,
                     const float min,
                     const float max) {
    derivative->elements = elements;
    derivative->dim = dim;
    derivative->min = min;
    derivative->max = max;

    derivative_reset(derivative);
}

void derivative_reset(derivative_t *derivative) {
    derivative->time_prev = task_timebase();

    for(uint32_t i = 0; i < derivative->dim; i++) {
        derivative->elements[i].value = 0;
        derivative->elements[i].value_prev = 0;
    }
}

void derivative_step(derivative_t *derivative, const float *input) {
    const uint32_t now = task_timebase();

    const float dt = (now - derivative->time_prev) * 0.000001f;

    for(uint32_t i = 0; i < derivative->dim; i++) {
        derivative->elements[i].value = (input[i] - derivative->elements[i].value_prev) / dt;

        if(derivative->elements[i].value < derivative->min) {
            derivative->elements[i].value = derivative->min;
        }

        if(derivative->elements[i].value > derivative->max) {
            derivative->elements[i].value = derivative->max;
        }

        derivative->elements[i].value_prev = input[i];
    }

    derivative->time_prev = now;
}

float derivative_get(const derivative_t *derivative, const uint32_t index) {
    return derivative->elements[index].value;
}
