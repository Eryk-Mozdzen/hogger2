#ifndef PID_H
#define PID_H

#include "control/derivative.h"
#include "control/integrator.h"

#define PID_INIT(_kp, _ki, _kd, _min, _max)                                                        \
    {                                                                                              \
        .kp = (_kp),                                                                               \
        .ki = (_ki),                                                                               \
        .kd = (_kd),                                                                               \
        .min = (_min),                                                                             \
        .max = (_max),                                                                             \
    }

typedef struct {
    const float kp;
    const float ki;
    const float kd;

    const float min;
    const float max;

    float output_unconstrained;
    float output_constrained;

    integrator_element_t integrator_element;
    integrator_t integrator;
    derivative_element_t derivative_element;
    derivative_t derivative;
} pid_t;

void pid_reset(pid_t *instance);
float pid_calculate(pid_t *instance, const float sp, const float pv);

#endif
