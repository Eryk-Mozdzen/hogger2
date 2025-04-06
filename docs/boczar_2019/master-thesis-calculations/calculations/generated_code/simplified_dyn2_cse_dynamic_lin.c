#include "simplified_dyn2_cse_dynamic_lin.h"


#include "robot_parameters.h"

void simplified_dyn2_cse_feedback_v(float v[5], const float K1[25], const float K2[25], const float q[9], const float h[5], const float d_h[5], const float hd[5], const float d_hd[5], const float d2_hd[5]) {
    float x0 = d_h[0] - d_hd[0];
    float x1 = d_h[1] - d_hd[1];
    float x2 = d_h[2] - d_hd[2];
    float x3 = d_h[3] - d_hd[3];
    float x4 = d_h[4] - d_hd[4];
    float x5 = hd[0] - h[0];
    float x6 = hd[1] - h[1];
    float x7 = hd[2] - h[2];
    float x8 = hd[3] - h[3];
    float x9 = hd[4] - h[4];
    
    v[0] = -K1[0]*x0 - K1[1]*x1 - K1[2]*x2 - K1[3]*x3 - K1[4]*x4 + K2[0]*x5 + K2[1]*x6 + K2[2]*x7 + K2[3]*x8 + K2[4]*x9 + d2_hd[0];
    v[1] = -K1[5]*x0 - K1[6]*x1 - K1[7]*x2 - K1[8]*x3 - K1[9]*x4 + K2[5]*x5 + K2[6]*x6 + K2[7]*x7 + K2[8]*x8 + K2[9]*x9 + d2_hd[1];
    v[2] = -K1[10]*x0 - K1[11]*x1 - K1[12]*x2 - K1[13]*x3 - K1[14]*x4 + K2[10]*x5 + K2[11]*x6 + K2[12]*x7 + K2[13]*x8 + K2[14]*x9 + d2_hd[2];
    v[3] = -K1[15]*x0 - K1[16]*x1 - K1[17]*x2 - K1[18]*x3 - K1[19]*x4 + K2[15]*x5 + K2[16]*x6 + K2[17]*x7 + K2[18]*x8 + K2[19]*x9 + d2_hd[3];
    v[4] = -K1[20]*x0 - K1[21]*x1 - K1[22]*x2 - K1[23]*x3 - K1[24]*x4 + K2[20]*x5 + K2[21]*x6 + K2[22]*x7 + K2[23]*x8 + K2[24]*x9 + d2_hd[4];
}
void simplified_dyn2_cse_dynamic_lin_u(float u[5], const float v[5], const float eta[5], const float q[9]) {
    float x0 = q[2];
    float x1 = q[3];
    float x2 = x0 + x1;
    float x3 = sinf(x2);
    float x4 = eta[1];
    float x5 = q[7];
    float x6 = q[5];
    float x7 = sinf(x6);
    float x8 = powf(x4, 2)*powf(x5, 2)*sinf(x1 - x6)/(l*x7);
    float x9 = 2*v[0] + x3*x8;
    float x10 = cosf(x2);
    float x11 = 2*v[1] - x10*x8;
    float x12 = 1.0F/x4;
    float x13 = x12/x5;
    float x14 = (1.0F/2.0F)*x9;
    float x15 = (1.0F/2.0F)*x11;
    float x16 = q[8];
    float x17 = sinf(x1);
    
    u[0] = (1.0F/2.0F)*x13*(x10*x11 - x3*x9);
    u[1] = v[3];
    u[2] = v[2];
    u[3] = x12*(-v[3]*x5 + x10*x14 + x15*x3);
    u[4] = x13*x16*(-x14*sinf(x0) + x15*cosf(x0) - x16*x7*(v[4] + x17*x4*x5*eta[2]*cosf(x6)/(x16*powf(x7, 2))))/x17;
}
float simplified_dyn2_cse_dynamic_lin_det_Kdd(const float q[], const float eta[]) {
    float det_Kdd = -powf(eta[1], 3)*powf(q[7], 2)*sinf(q[3])/(powf(q[8], 2)*sinf(q[5]));
    return det_Kdd;
}
