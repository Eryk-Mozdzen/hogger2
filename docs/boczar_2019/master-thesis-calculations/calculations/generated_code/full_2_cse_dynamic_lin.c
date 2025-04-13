#include "full_2_cse_dynamic_lin.h"


#include "robot_parameters.h"

void full_2_cse_feedback_v(float v[5], const float K1[25], const float K2[25], const float q[9], const float h[5], const float d_h[5], const float hd[5], const float d_hd[5], const float d2_hd[5]) {
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
void full_2_cse_dynamic_lin_u(float u[5], const float v[5], const float eta[5], const float q[9]) {
    float x0 = q[4];
    float x1 = sinf(x0);
    float x2 = q[2];
    float x3 = sinf(x2);
    float x4 = 2*l;
    float x5 = eta[0];
    float x6 = eta[1];
    float x7 = q[3];
    float x8 = sinf(x7);
    float x9 = x6*x8;
    float x10 = x5*x9;
    float x11 = x10*x4;
    float x12 = cosf(x7);
    float x13 = x12*x6;
    float x14 = eta[3];
    float x15 = q[6];
    float x16 = cosf(x15);
    float x17 = sinf(x15);
    float x18 = eta[4];
    float x19 = q[7];
    float x20 = cosf(x19);
    float x21 = x18*x20;
    float x22 = eta[2];
    float x23 = cosf(x0);
    float x24 = x22*x23;
    float x25 = R*(x13 - x14*x16 + x17*x21 - x24*x8);
    float x26 = x3*x5;
    float x27 = cosf(x2);
    float x28 = x25*x27;
    float x29 = x23*x6;
    float x30 = x12*x23;
    float x31 = x1*x3;
    float x32 = x23*x8;
    float x33 = R/l;
    float x34 = 2*v[1] + x33*(x11*x3 + x13*x28 + x22*(x25*x31 - x28*x32 + x4*(x26*x30 + x27*x29 - x31*x9)) + x25*x26);
    float x35 = 1.0F/R;
    float x36 = (1.0F/2.0F)*x35;
    float x37 = x27*x36;
    float x38 = 2*v[0];
    float x39 = x11*x27;
    float x40 = x27*x5;
    float x41 = x25*x40;
    float x42 = x25*x3;
    float x43 = x13*x42;
    float x44 = x1*x27;
    float x45 = x22*(x25*x44 + x32*x42 + x4*(-x29*x3 + x30*x40 - x44*x9));
    float x46 = x3*x36;
    float x47 = 1.0F/x12;
    float x48 = x34*x46;
    float x49 = x37*(x33*(x39 + x41 - x43 + x45) + x38);
    float x50 = 1.0F/x16;
    float x51 = x1*x22;
    float x52 = x18*sinf(x19);
    float x53 = x14*x17;
    float x54 = x5 + x51 - x52;
    
    u[0] = -v[3]*x1 - x34*x37 - x46*(x33*(-x39 - x41 + x43 - x45) - x38);
    u[1] = v[3]*x23*tanf(x7) + x47*x48 + x47*x49;
    u[2] = v[3];
    u[3] = l*x35*x50*(2*v[2] + x33*(-x10 - x12*x24*x5 + x16*x21*x54 + x51*x9 - x52*x53 + x53*x54)) + v[4]*x20*tanf(x15) + x48*x50 + x49*x50;
    u[4] = v[4];
}
float full_2_cse_dynamic_lin_det_Kdd(const float q[], const float eta[]) {
    float det_Kdd = -1.0F/2.0F*powf(R, 3)*cosf(q[3])*cosf(q[6])/l;
    return det_Kdd;
}
