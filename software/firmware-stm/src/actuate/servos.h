#ifndef SERVOS_H
#define SERVOS_H

#include <stdbool.h>

void servos_set_position(const float phi_1,
                         const float theta_1,
                         const float phi_2,
                         const float theta_2);

#endif
