#include "full_1_cse_dynamic_lin.h"


#include "robot_parameters.h"

void full_1_cse_feedback_v(float v[5], const float K1[25], const float K2[25], const float q[9], const float h[5], const float d_h[5], const float hd[5], const float d_hd[5], const float d2_hd[5]) {
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
void full_1_cse_dynamic_lin_u(float u[5], const float v[5], const float eta[5], const float q[9]) {
    float x0 = q[4];
    float x1 = sinf(x0);
    float x2 = q[2];
    float x3 = sinf(x2);
    float x4 = eta[0];
    float x5 = eta[1];
    float x6 = q[3];
    float x7 = sinf(x6);
    float x8 = x5*x7;
    float x9 = x4*x8;
    float x10 = q[7];
    float x11 = tanf(x10);
    float x12 = 2*l*x11;
    float x13 = x12*x9;
    float x14 = eta[3];
    float x15 = q[6];
    float x16 = sinf(x15);
    float x17 = x14*x16;
    float x18 = eta[4];
    float x19 = cosf(x15);
    float x20 = cosf(x6);
    float x21 = x20*x5;
    float x22 = eta[2];
    float x23 = x1*x16;
    float x24 = cosf(x0);
    float x25 = x24*x7;
    float x26 = R*(x11*(x18*x19 - x21) - x16*x4 + x17 + x22*(x11*x25 - x23));
    float x27 = x26*x3;
    float x28 = cosf(x2);
    float x29 = x26*x28;
    float x30 = x24*x5;
    float x31 = x20*x24*x4;
    float x32 = x1*x8;
    float x33 = R/l;
    float x34 = x33/x11;
    float x35 = -2*v[1] + x34*(-x13*x3 + x21*x29 - x22*(-x1*x27 + x12*(x28*x30 + x3*x31 - x3*x32) + x25*x29) + x27*x4);
    float x36 = 1.0F/R;
    float x37 = (1.0F/2.0F)*x36;
    float x38 = x28*x37;
    float x39 = -2*v[0] + x34*(-x13*x28 - x21*x27 + x22*(x1*x29 + x12*(-x28*x31 + x28*x32 + x3*x30) + x25*x27) + x29*x4);
    float x40 = x3*x37;
    float x41 = x35*x38 - x39*x40;
    float x42 = 1.0F/x20;
    float x43 = x38*x39;
    float x44 = x35*x40;
    float x45 = sinf(x10);
    float x46 = cosf(x10);
    float x47 = -x14 + x4;
    float x48 = v[4] - (-x18*x46*(x1*x22 + x47) + x22*x30*x45)/powf(x45, 2);
    float x49 = 1.0F/x19;
    float x50 = powf(x11, 2);
    float x51 = x14*x19;
    float x52 = x18/powf(x46, 2);
    
    u[0] = -v[3]*x1 + x41;
    u[1] = v[3]*x24*tanf(x6) - x42*x43 - x42*x44;
    u[2] = v[3];
    u[3] = x41 - x45*x48;
    u[4] = -l*x36*x49*(-2*v[2] + x33*(x11*x51*(x14 - x4) + x16*x47*x52 - x22*(x11*(x1*x51 + x16*x30) - x23*x52 + x50*(-x31 + x32)) + x50*(-x17*x18 + x9))/x50) - x43*x49 - x44*x49 + x46*x48*tanf(x15);
}
float full_1_cse_dynamic_lin_det_Kdd(const float q[], const float eta[]) {
    float det_Kdd = -1.0F/2.0F*powf(R, 3)*cosf(q[3])*cosf(q[6])/(l*sinf(q[7]));
    return det_Kdd;
}
