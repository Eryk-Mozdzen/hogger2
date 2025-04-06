#include "simplified_conversions.h"


#include "robot_parameters.h"

void simplified_full_to_simplified(float q_simp[9], const float q_full[9]) {
    
    q_simp[0] = q_simp[0];
    q_simp[1] = q_simp[1];
    q_simp[2] = q_simp[2];
    q_simp[3] = atan2f(sinf(q_full[4]), sinf(q_full[3])*cosf(q_full[4]));
    q_simp[4] = q_full[5];
    q_simp[5] = atan2f(sinf(q_full[7]), sinf(q_full[6])*cosf(q_full[7]));
    q_simp[6] = q_full[8];
    q_simp[7] = R*sqrtf((powf(sinf(q_full[4]), 2) - 1)*powf(cosf(q_full[3]), 2) + 1);
    q_simp[8] = R*sqrtf((powf(sinf(q_full[7]), 2) - 1)*powf(cosf(q_full[6]), 2) + 1);
}
void simplified_simplified_to_full(float q_full[9], const float q_simp[9]) {
    
    q_full[0] = q_simp[0];
    q_full[1] = q_simp[1];
    q_full[2] = q_simp[2];
    q_full[3] = ((fmodf(q_simp[3], M_PI) > (3.0F/2.0F)*M_PI || fmodf(q_simp[3], M_PI) < M_PI_2) ? (
       1
    )
    : (
       -1
    ))*acosf(sqrtf(-powf(R, 2) + powf(q_simp[7], 2))*sqrtf(powf(tanf(q_simp[3]), 2) + 1)/sqrtf(-powf(R, 2)*powf(tanf(q_simp[3]), 2) - powf(R, 2) + powf(q_simp[7], 2)*powf(tanf(q_simp[3]), 2)));
    q_full[4] = ((fmodf(q_simp[3], M_PI) > (3.0F/2.0F)*M_PI || fmodf(q_simp[3], M_PI) < M_PI_2) ? (
       1
    )
    : (
       -1
    ))*asinf(q_simp[7]*tanf(q_simp[3])/(R*sqrtf(powf(tanf(q_simp[3]), 2) + 1)));
    q_full[5] = q_simp[4];
    q_full[6] = ((fmodf(q_simp[5], M_PI) > (3.0F/2.0F)*M_PI || fmodf(q_simp[5], M_PI) < M_PI_2) ? (
       1
    )
    : (
       -1
    ))*acosf(sqrtf(-powf(R, 2) + powf(q_simp[8], 2))*sqrtf(powf(tanf(q_simp[5]), 2) + 1)/sqrtf(-powf(R, 2)*powf(tanf(q_simp[5]), 2) - powf(R, 2) + powf(q_simp[8], 2)*powf(tanf(q_simp[5]), 2)));
    q_full[7] = ((fmodf(q_simp[5], M_PI) > (3.0F/2.0F)*M_PI || fmodf(q_simp[5], M_PI) < M_PI_2) ? (
       1
    )
    : (
       -1
    ))*asinf(q_simp[8]*tanf(q_simp[5])/(R*sqrtf(powf(tanf(q_simp[5]), 2) + 1)));
    q_full[8] = q_simp[6];
}
