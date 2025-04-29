#include "control/integrator.h"
#include "utils/task.h"

void integrator_init(integrator_t *integrator,
                     integrator_element_t *elements,
                     const uint32_t dim,
                     const float min,
                     const float max) {
    integrator->elements = elements;
    integrator->dim = dim;
    integrator->min = min;
    integrator->max = max;

    integrator_reset(integrator);
}

void integrator_reset(integrator_t *integrator) {
    integrator->time_prev = task_timebase();

    for(uint32_t i = 0; i < integrator->dim; i++) {
        integrator->elements[i].value = 0;
        integrator->elements[i].value_prev = 0;
    }
}

void integrator_step(integrator_t *integrator, const float *input) {
    const uint32_t now = task_timebase();

    const float dt = (now - integrator->time_prev) * 0.000001f;

    for(uint32_t i = 0; i < integrator->dim; i++) {
        integrator->elements[i].value +=
            (0.5f * (input[i] + integrator->elements[i].value_prev) * dt);

        if(integrator->elements[i].value < integrator->min) {
            integrator->elements[i].value = integrator->min;
        }

        if(integrator->elements[i].value > integrator->max) {
            integrator->elements[i].value = integrator->max;
        }

        integrator->elements[i].value_prev = input[i];
    }

    integrator->time_prev = now;
}

void integrator_set(integrator_t *integrator, const uint32_t index, const float value) {
    integrator->elements[index].value = value;
    integrator->elements[index].value_prev = value;
}

float integrator_get(const integrator_t *integrator, const uint32_t index) {
    return integrator->elements[index].value;
}
