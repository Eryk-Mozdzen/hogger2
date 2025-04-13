#ifndef MOTORS_H
#define MOTORS_H

void motors_set_velocity(const float psi1_dot, const float psi2_dot);
void motors_get_velocity(float *psi1_dot, float *psi2_dot);

#endif
