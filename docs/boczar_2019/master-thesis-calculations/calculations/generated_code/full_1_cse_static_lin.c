#include "full_1_cse_static_lin.h"


#include "robot_parameters.h"

void full_1_cse_feedback_u(float u[5], const float K[25], const float q[9], const float h[5], const float hd[5], const float d_hd[5]) {
    float x0 = hd[0] - h[0];
    float x1 = hd[1] - h[1];
    float x2 = hd[2] - h[2];
    float x3 = hd[3] - h[3];
    float x4 = hd[4] - h[4];
    
    u[0] = K[0]*x0 + K[1]*x1 + K[2]*x2 + K[3]*x3 + K[4]*x4 + d_hd[0];
    u[1] = K[5]*x0 + K[6]*x1 + K[7]*x2 + K[8]*x3 + K[9]*x4 + d_hd[1];
    u[2] = K[10]*x0 + K[11]*x1 + K[12]*x2 + K[13]*x3 + K[14]*x4 + d_hd[2];
    u[3] = K[15]*x0 + K[16]*x1 + K[17]*x2 + K[18]*x3 + K[19]*x4 + d_hd[3];
    u[4] = K[20]*x0 + K[21]*x1 + K[22]*x2 + K[23]*x3 + K[24]*x4 + d_hd[4];
}
void full_1_cse_static_lin_eta(float eta[5], const float u[5], const float q[9]) {
    float x0 = q[4];
    float x1 = q[2];
    float x2 = sinf(x1);
    float x3 = 1.0F/R;
    float x4 = u[0]*x3;
    float x5 = cosf(x1);
    float x6 = u[1]*x3;
    float x7 = x2*x4 - x5*x6;
    float x8 = q[3];
    float x9 = 1.0F/cosf(x8);
    float x10 = x4*x5;
    float x11 = x2*x6;
    float x12 = q[7];
    float x13 = q[6];
    float x14 = 1.0F/cosf(x13);
    
    eta[0] = -u[3]*sinf(x0) + x7;
    eta[1] = u[3]*cosf(x0)*tanf(x8) + x10*x9 + x11*x9;
    eta[2] = u[3];
    eta[3] = -u[4]*sinf(x12) + x7;
    eta[4] = 2*l*u[2]*x14*x3 + u[4]*cosf(x12)*tanf(x13) + x10*x14 + x11*x14;
}
float full_1_cse_static_lin_det_D(const float q[]) {
    float det_D = -1.0F/2.0F*powf(R, 3)*cosf(q[3])*cosf(q[6])/(l*sinf(q[7]));
    return det_D;
}
