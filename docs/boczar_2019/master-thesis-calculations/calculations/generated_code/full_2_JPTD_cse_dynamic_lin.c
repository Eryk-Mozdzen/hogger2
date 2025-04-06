#include "full_2_JPTD_cse_dynamic_lin.h"


#include "robot_parameters.h"

void full_2_JPTD_cse_feedback_v(float v[5], const float K1[25], const float K2[25], const float q[9], const float h[5], const float d_h[5], const float hd[5], const float d_hd[5], const float d2_hd[5]) {
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
void full_2_JPTD_cse_dynamic_lin_u(float u[5], const float v[5], const float eta[5], const float q[9]) {
    float x0 = q[4];
    float x1 = cosf(x0);
    float x2 = 1.0F/x1;
    float x3 = q[3];
    float x4 = tanf(x3);
    float x5 = v[3]*x4;
    float x6 = q[2];
    float x7 = sinf(x6);
    float x8 = 1.0F/cosf(x3);
    float x9 = x7*x8;
    float x10 = cosf(x6);
    float x11 = tanf(x0);
    float x12 = x11*x4;
    float x13 = sinf(x0);
    float x14 = q[6];
    float x15 = sinf(x14);
    float x16 = eta[4];
    float x17 = q[7];
    float x18 = x16*cosf(x17);
    float x19 = x15*x18;
    float x20 = x19*x7;
    float x21 = eta[2];
    float x22 = powf(x1, 2);
    float x23 = sinf(x3);
    float x24 = x21*x22*powf(x23, 2);
    float x25 = x13*x21;
    float x26 = x1*x23;
    float x27 = x25*x26;
    float x28 = 1.0F/l;
    float x29 = powf(R, 2)*x21*x28;
    float x30 = 1.0F/R;
    float x31 = (1.0F/2.0F)*x30;
    float x32 = x31*(2*v[1] + x29*(-x10*x19*x26 + x10*x24 + x13*x20 - x27*x7));
    float x33 = x10*x8;
    float x34 = x10*x13;
    float x35 = x31*(-2*v[0] + x29*(x10*x27 - x19*x34 - x20*x26 + x24*x7));
    float x36 = 1.0F/x21;
    float x37 = x2*x36;
    float x38 = 1.0F/x16;
    float x39 = 1.0F/tanf(x17);
    float x40 = x39/tanf(x14);
    float x41 = x36*x40/x22;
    float x42 = sinf(x17);
    float x43 = x38/(x15*x42);
    float x44 = x37*x40;
    float x45 = x4*x41;
    
    u[0] = -x37*(x2*x5 + x32*(x10*x12 + x9) + x35*(x12*x7 - x33));
    u[1] = -x36*(v[3]*x11 + x10*x2*x32 + x2*x35*x7);
    u[2] = v[3];
    u[3] = l*x30*x43*(R*x18*x28*(-x16*x42 + x25)*cosf(x14) + 2*v[2]) + v[4]*x38*x39 - x32*(x34*x45 - x43*x7 + x44*x9) - x35*(x10*x43 + x13*x45*x7 - x33*x44) - x41*x5;
    u[4] = v[4];
}
float full_2_JPTD_cse_dynamic_lin_det_Kdd(const float q[], const float eta[]) {
    float det_Kdd = -1.0F/2.0F*powf(R, 3)*powf(eta[2], 2)*eta[4]*sinf(q[6])*sinf(q[7])*cosf(q[3])*powf(cosf(q[4]), 2)/l;
    return det_Kdd;
}
