#pragma once

#include <math.h>

void full_2_cse_feedback_u(float u[5], const float K[25], const float q[9], const float h[5], const float hd[5], const float d_hd[5]);
void full_2_cse_static_lin_eta(float eta[5], const float u[5], const float q[9]);
float full_2_cse_static_lin_det_D(const float q[]);
