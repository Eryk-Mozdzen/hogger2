#pragma once

#include <math.h>

void full_2_JPTD_cse_feedback_v(float v[5], const float K1[25], const float K2[25], const float q[9], const float h[5], const float d_h[5], const float hd[5], const float d_hd[5], const float d2_hd[5]);
void full_2_JPTD_cse_dynamic_lin_u(float u[5], const float v[5], const float eta[5], const float q[9]);
float full_2_JPTD_cse_dynamic_lin_det_Kdd(const float q[], const float eta[]);
