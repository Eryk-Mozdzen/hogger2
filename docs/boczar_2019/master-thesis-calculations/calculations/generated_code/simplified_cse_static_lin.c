#include "simplified_cse_static_lin.h"


#include "robot_parameters.h"

void simplified_cse_feedback_u(float u[5], const float K[25], const float q[9], const float h[5], const float hd[5], const float d_hd[5]) {
    float x0 = hd[2] - h[2];
    float x1 = hd[3] - h[3];
    float x2 = hd[4] - h[4];
    float x3 = q[2] + q[3];
    float x4 = d*cosf(x3) - hd[0] + q[0];
    float x5 = d*sinf(x3) - hd[1] + q[1];
    
    u[0] = -K[0]*x4 - K[1]*x5 + K[2]*x0 + K[3]*x1 + K[4]*x2 + d_hd[0];
    u[1] = -K[5]*x4 - K[6]*x5 + K[7]*x0 + K[8]*x1 + K[9]*x2 + d_hd[1];
    u[2] = -K[10]*x4 - K[11]*x5 + K[12]*x0 + K[13]*x1 + K[14]*x2 + d_hd[2];
    u[3] = -K[15]*x4 - K[16]*x5 + K[17]*x0 + K[18]*x1 + K[19]*x2 + d_hd[3];
    u[4] = -K[20]*x4 - K[21]*x5 + K[22]*x0 + K[23]*x1 + K[24]*x2 + d_hd[4];
}
void simplified_cse_static_lin_eta(float eta[5], const float u[5], const float q[9]) {
    float x0 = q[3];
    float x1 = x0 + q[2];
    float x2 = sinf(x1);
    float x3 = 2/d;
    float x4 = cosf(x1);
    float x5 = q[5];
    float x6 = sinf(x0 - x5)/(l*sinf(x5));
    
    eta[0] = -1.0F/2.0F*u[0]*(x2*x3 + x4*x6) - 1.0F/2.0F*u[1]*(x2*x6 - x3*x4);
    eta[1] = (u[0]*x4 + u[1]*x2)/q[7];
    eta[2] = u[2];
    eta[3] = u[3];
    eta[4] = u[4];
}
float simplified_cse_static_lin_det_D(const float q[]) {
    float det_D = -d*q[7];
    return det_D;
}
