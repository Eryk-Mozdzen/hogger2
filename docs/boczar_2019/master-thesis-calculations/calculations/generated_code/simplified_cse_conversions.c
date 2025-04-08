#include "simplified_cse_conversions.h"


#include "robot_parameters.h"

void simplified_cse_full_to_simplified(float q_simp[9], const float q_full[9]) {
    float x0 = q_full[4];
    float x1 = sinf(x0);
    float x2 = q_full[3];
    float x3 = q_full[7];
    float x4 = sinf(x3);
    float x5 = q_full[6];
    
    q_simp[0] = q_simp[0];
    q_simp[1] = q_simp[1];
    q_simp[2] = q_simp[2];
    q_simp[3] = atan2f(x1, sinf(x2)*cosf(x0));
    q_simp[4] = q_full[5];
    q_simp[5] = atan2f(x4, sinf(x5)*cosf(x3));
    q_simp[6] = q_full[8];
    q_simp[7] = R*sqrtf((powf(x1, 2) - 1)*powf(cosf(x2), 2) + 1);
    q_simp[8] = R*sqrtf((powf(x4, 2) - 1)*powf(cosf(x5), 2) + 1);
}
void simplified_cse_simplified_to_full(float q_full[9], const float q_simp[9]) {
    float x0 = q_simp[3];
    float x1 = fmodf(x0, M_PI);
    float x2 = (3.0F/2.0F)*M_PI;
    float x3 = M_PI_2;
    float x4 = ((x1 > x2 || x1 < x3) ? (
   1
)
: (
   -1
));
    float x5 = tanf(x0);
    float x6 = powf(x5, 2);
    float x7 = sqrtf(x6 + 1);
    float x8 = q_simp[7];
    float x9 = powf(x8, 2);
    float x10 = powf(R, 2);
    float x11 = -x10;
    float x12 = 1.0F/R;
    float x13 = q_simp[5];
    float x14 = fmodf(x13, M_PI);
    float x15 = ((x14 > x2 || x14 < x3) ? (
   1
)
: (
   -1
));
    float x16 = tanf(x13);
    float x17 = powf(x16, 2);
    float x18 = sqrtf(x17 + 1);
    float x19 = q_simp[8];
    float x20 = powf(x19, 2);
    
    q_full[0] = q_simp[0];
    q_full[1] = q_simp[1];
    q_full[2] = q_simp[2];
    q_full[3] = x4*acosf(x7*sqrtf(x11 + x9)/sqrtf(-x10*x6 + x11 + x6*x9));
    q_full[4] = x4*asinf(x12*x5*x8/x7);
    q_full[5] = q_simp[4];
    q_full[6] = x15*acosf(x18*sqrtf(x11 + x20)/sqrtf(-x10*x17 + x11 + x17*x20));
    q_full[7] = x15*asinf(x12*x16*x19/x18);
    q_full[8] = q_simp[6];
}
