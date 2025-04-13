#include "simplified_dyn1_dynamic_lin.h"


#include "robot_parameters.h"

void simplified_dyn1_feedback_v(float v[5], const float K1[25], const float K2[25], const float q[9], const float h[5], const float d_h[5], const float hd[5], const float d_hd[5], const float d2_hd[5]) {
    
    v[0] = -K1[0]*(d_h[0] - d_hd[0]) - K1[1]*(d_h[1] - d_hd[1]) - K1[2]*(d_h[2] - d_hd[2]) - K1[3]*(d_h[3] - d_hd[3]) - K1[4]*(d_h[4] - d_hd[4]) - K2[0]*(-hd[0] + h[0]) - K2[1]*(-hd[1] + h[1]) - K2[2]*(-hd[2] + h[2]) - K2[3]*(-hd[3] + h[3]) - K2[4]*(-hd[4] + h[4]) + d2_hd[0];
    v[1] = -K1[5]*(d_h[0] - d_hd[0]) - K1[6]*(d_h[1] - d_hd[1]) - K1[7]*(d_h[2] - d_hd[2]) - K1[8]*(d_h[3] - d_hd[3]) - K1[9]*(d_h[4] - d_hd[4]) - K2[5]*(-hd[0] + h[0]) - K2[6]*(-hd[1] + h[1]) - K2[7]*(-hd[2] + h[2]) - K2[8]*(-hd[3] + h[3]) - K2[9]*(-hd[4] + h[4]) + d2_hd[1];
    v[2] = -K1[10]*(d_h[0] - d_hd[0]) - K1[11]*(d_h[1] - d_hd[1]) - K1[12]*(d_h[2] - d_hd[2]) - K1[13]*(d_h[3] - d_hd[3]) - K1[14]*(d_h[4] - d_hd[4]) - K2[10]*(-hd[0] + h[0]) - K2[11]*(-hd[1] + h[1]) - K2[12]*(-hd[2] + h[2]) - K2[13]*(-hd[3] + h[3]) - K2[14]*(-hd[4] + h[4]) + d2_hd[2];
    v[3] = -K1[15]*(d_h[0] - d_hd[0]) - K1[16]*(d_h[1] - d_hd[1]) - K1[17]*(d_h[2] - d_hd[2]) - K1[18]*(d_h[3] - d_hd[3]) - K1[19]*(d_h[4] - d_hd[4]) - K2[15]*(-hd[0] + h[0]) - K2[16]*(-hd[1] + h[1]) - K2[17]*(-hd[2] + h[2]) - K2[18]*(-hd[3] + h[3]) - K2[19]*(-hd[4] + h[4]) + d2_hd[3];
    v[4] = -K1[20]*(d_h[0] - d_hd[0]) - K1[21]*(d_h[1] - d_hd[1]) - K1[22]*(d_h[2] - d_hd[2]) - K1[23]*(d_h[3] - d_hd[3]) - K1[24]*(d_h[4] - d_hd[4]) - K2[20]*(-hd[0] + h[0]) - K2[21]*(-hd[1] + h[1]) - K2[22]*(-hd[2] + h[2]) - K2[23]*(-hd[3] + h[3]) - K2[24]*(-hd[4] + h[4]) + d2_hd[4];
}
void simplified_dyn1_dynamic_lin_u(float u[5], const float v[5], const float eta[5], const float q[9]) {
    
    u[0] = -(v[0] - eta[1]*eta[3]*cosf(q[2] + q[3]) + (1.0F/2.0F)*powf(eta[1], 2)*powf(q[7], 2)*sinf(q[2] + q[3])*sinf(q[3] - q[5])/(l*sinf(q[5])))*sinf(q[2] + q[3])/(eta[1]*q[7]) + (v[1] - eta[1]*eta[3]*sinf(q[2] + q[3]) - 1.0F/2.0F*powf(eta[1], 2)*powf(q[7], 2)*sinf(q[3] - q[5])*cosf(q[2] + q[3])/(l*sinf(q[5])))*cosf(q[2] + q[3])/(eta[1]*q[7]);
    u[1] = (v[0] - eta[1]*eta[3]*cosf(q[2] + q[3]) + (1.0F/2.0F)*powf(eta[1], 2)*powf(q[7], 2)*sinf(q[2] + q[3])*sinf(q[3] - q[5])/(l*sinf(q[5])))*cosf(q[2] + q[3])/q[7] + (v[1] - eta[1]*eta[3]*sinf(q[2] + q[3]) - 1.0F/2.0F*powf(eta[1], 2)*powf(q[7], 2)*sinf(q[3] - q[5])*cosf(q[2] + q[3])/(l*sinf(q[5])))*sinf(q[2] + q[3])/q[7];
    u[2] = v[2];
    u[3] = v[3];
    u[4] = v[4];
}
float simplified_dyn1_dynamic_lin_det_Kdd(const float q[], const float eta[]) {
    float det_Kdd = -eta[1]*powf(q[7], 2);
    return det_Kdd;
}
