#include "control/pid.h"
#include "control/derivative.h"
#include "control/integrator.h"

#define K_ANTIWINDUP 1.6f

void pid_reset(pid_t *instance) {
    instance->output_unconstrained = 0;
    instance->output_constrained = 0;

    integrator_init(&instance->integrator, &instance->integrator_element, 1, -1000, 1000);
    derivative_init(&instance->derivative, &instance->derivative_element, 1, -1000, 1000);
}

float pid_calculate(pid_t *instance, const float sp, const float pv) {
    const float error = sp - pv;

    const float error_and_antiwindup =
        error + (instance->output_constrained - instance->output_unconstrained) * K_ANTIWINDUP;

    integrator_step(&instance->integrator, &error_and_antiwindup);
    derivative_step(&instance->derivative, &error);

    const float error_integral = integrator_get(&instance->integrator, 0);
    const float error_derivative = derivative_get(&instance->derivative, 0);

    instance->output_unconstrained = (instance->kp * error) + (instance->ki * error_integral) +
                                     (instance->kd * error_derivative);

    if(instance->output_unconstrained > instance->max) {
        instance->output_constrained = instance->max;
    } else if(instance->output_unconstrained < instance->min) {
        instance->output_constrained = instance->min;
    } else {
        instance->output_constrained = instance->output_unconstrained;
    }

    return instance->output_constrained;
}
