#include "full_2_JPTD_dynamic_lin.h"


#include "robot_parameters.h"

void full_2_JPTD_feedback_v(float v[5], const float K1[25], const float K2[25], const float q[9], const float h[5], const float d_h[5], const float hd[5], const float d_hd[5], const float d2_hd[5]) {
    
    v[0] = -K1[0]*(d_h[0] - d_hd[0]) - K1[1]*(d_h[1] - d_hd[1]) - K1[2]*(d_h[2] - d_hd[2]) - K1[3]*(d_h[3] - d_hd[3]) - K1[4]*(d_h[4] - d_hd[4]) - K2[0]*(-hd[0] + h[0]) - K2[1]*(-hd[1] + h[1]) - K2[2]*(-hd[2] + h[2]) - K2[3]*(-hd[3] + h[3]) - K2[4]*(-hd[4] + h[4]) + d2_hd[0];
    v[1] = -K1[5]*(d_h[0] - d_hd[0]) - K1[6]*(d_h[1] - d_hd[1]) - K1[7]*(d_h[2] - d_hd[2]) - K1[8]*(d_h[3] - d_hd[3]) - K1[9]*(d_h[4] - d_hd[4]) - K2[5]*(-hd[0] + h[0]) - K2[6]*(-hd[1] + h[1]) - K2[7]*(-hd[2] + h[2]) - K2[8]*(-hd[3] + h[3]) - K2[9]*(-hd[4] + h[4]) + d2_hd[1];
    v[2] = -K1[10]*(d_h[0] - d_hd[0]) - K1[11]*(d_h[1] - d_hd[1]) - K1[12]*(d_h[2] - d_hd[2]) - K1[13]*(d_h[3] - d_hd[3]) - K1[14]*(d_h[4] - d_hd[4]) - K2[10]*(-hd[0] + h[0]) - K2[11]*(-hd[1] + h[1]) - K2[12]*(-hd[2] + h[2]) - K2[13]*(-hd[3] + h[3]) - K2[14]*(-hd[4] + h[4]) + d2_hd[2];
    v[3] = -K1[15]*(d_h[0] - d_hd[0]) - K1[16]*(d_h[1] - d_hd[1]) - K1[17]*(d_h[2] - d_hd[2]) - K1[18]*(d_h[3] - d_hd[3]) - K1[19]*(d_h[4] - d_hd[4]) - K2[15]*(-hd[0] + h[0]) - K2[16]*(-hd[1] + h[1]) - K2[17]*(-hd[2] + h[2]) - K2[18]*(-hd[3] + h[3]) - K2[19]*(-hd[4] + h[4]) + d2_hd[3];
    v[4] = -K1[20]*(d_h[0] - d_hd[0]) - K1[21]*(d_h[1] - d_hd[1]) - K1[22]*(d_h[2] - d_hd[2]) - K1[23]*(d_h[3] - d_hd[3]) - K1[24]*(d_h[4] - d_hd[4]) - K2[20]*(-hd[0] + h[0]) - K2[21]*(-hd[1] + h[1]) - K2[22]*(-hd[2] + h[2]) - K2[23]*(-hd[3] + h[3]) - K2[24]*(-hd[4] + h[4]) + d2_hd[4];
}
void full_2_JPTD_dynamic_lin_u(float u[5], const float v[5], const float eta[5], const float q[9]) {
    
    u[0] = -v[3]*tanf(q[3])/(eta[2]*powf(cosf(q[4]), 2)) - (sinf(q[2])/cosf(q[3]) + cosf(q[2])*tanf(q[3])*tanf(q[4]))*((1.0F/2.0F)*powf(R, 2)*(eta[2]*powf(sinf(q[3]), 2)*cosf(q[2])*powf(cosf(q[4]), 2) - eta[2]*sinf(q[3])*sinf(q[2])*sinf(q[4])*cosf(q[4]) - eta[4]*sinf(q[3])*sinf(q[6])*cosf(q[2])*cosf(q[4])*cosf(q[7]) + eta[4]*sinf(q[6])*sinf(q[2])*sinf(q[4])*cosf(q[7]))*eta[2]/l + v[1])/(R*eta[2]*cosf(q[4])) + (sinf(q[2])*tanf(q[3])*tanf(q[4]) - cosf(q[2])/cosf(q[3]))*(-1.0F/2.0F*powf(R, 2)*(eta[2]*powf(sinf(q[3]), 2)*sinf(q[2])*powf(cosf(q[4]), 2) + eta[2]*sinf(q[3])*sinf(q[4])*cosf(q[2])*cosf(q[4]) - eta[4]*sinf(q[3])*sinf(q[6])*sinf(q[2])*cosf(q[4])*cosf(q[7]) - eta[4]*sinf(q[6])*sinf(q[4])*cosf(q[2])*cosf(q[7]))*eta[2]/l + v[0])/(R*eta[2]*cosf(q[4]));
    u[1] = -v[3]*tanf(q[4])/eta[2] + (-1.0F/2.0F*powf(R, 2)*(eta[2]*powf(sinf(q[3]), 2)*sinf(q[2])*powf(cosf(q[4]), 2) + eta[2]*sinf(q[3])*sinf(q[4])*cosf(q[2])*cosf(q[4]) - eta[4]*sinf(q[3])*sinf(q[6])*sinf(q[2])*cosf(q[4])*cosf(q[7]) - eta[4]*sinf(q[6])*sinf(q[4])*cosf(q[2])*cosf(q[7]))*eta[2]/l + v[0])*sinf(q[2])/(R*eta[2]*cosf(q[4])) - ((1.0F/2.0F)*powf(R, 2)*(eta[2]*powf(sinf(q[3]), 2)*cosf(q[2])*powf(cosf(q[4]), 2) - eta[2]*sinf(q[3])*sinf(q[2])*sinf(q[4])*cosf(q[4]) - eta[4]*sinf(q[3])*sinf(q[6])*cosf(q[2])*cosf(q[4])*cosf(q[7]) + eta[4]*sinf(q[6])*sinf(q[2])*sinf(q[4])*cosf(q[7]))*eta[2]/l + v[1])*cosf(q[2])/(R*eta[2]*cosf(q[4]));
    u[2] = v[3];
    u[3] = -v[3]*tanf(q[3])/(eta[2]*powf(cosf(q[4]), 2)*tanf(q[6])*tanf(q[7])) + v[4]/(eta[4]*tanf(q[7])) + (-1.0F/2.0F*powf(R, 2)*(eta[2]*powf(sinf(q[3]), 2)*sinf(q[2])*powf(cosf(q[4]), 2) + eta[2]*sinf(q[3])*sinf(q[4])*cosf(q[2])*cosf(q[4]) - eta[4]*sinf(q[3])*sinf(q[6])*sinf(q[2])*cosf(q[4])*cosf(q[7]) - eta[4]*sinf(q[6])*sinf(q[4])*cosf(q[2])*cosf(q[7]))*eta[2]/l + v[0])*(cosf(q[2])/(R*eta[4]*sinf(q[6])*sinf(q[7])) + sinf(q[2])*sinf(q[4])*tanf(q[3])/(R*eta[2]*powf(cosf(q[4]), 2)*tanf(q[6])*tanf(q[7])) - cosf(q[2])/(R*eta[2]*cosf(q[3])*cosf(q[4])*tanf(q[6])*tanf(q[7]))) + ((1.0F/2.0F)*powf(R, 2)*(eta[2]*powf(sinf(q[3]), 2)*cosf(q[2])*powf(cosf(q[4]), 2) - eta[2]*sinf(q[3])*sinf(q[2])*sinf(q[4])*cosf(q[4]) - eta[4]*sinf(q[3])*sinf(q[6])*cosf(q[2])*cosf(q[4])*cosf(q[7]) + eta[4]*sinf(q[6])*sinf(q[2])*sinf(q[4])*cosf(q[7]))*eta[2]/l + v[1])*(sinf(q[2])/(R*eta[4]*sinf(q[6])*sinf(q[7])) - sinf(q[2])/(R*eta[2]*cosf(q[3])*cosf(q[4])*tanf(q[6])*tanf(q[7])) - sinf(q[4])*cosf(q[2])*tanf(q[3])/(R*eta[2]*powf(cosf(q[4]), 2)*tanf(q[6])*tanf(q[7]))) + 2*l*((1.0F/2.0F)*R*(eta[2]*sinf(q[4]) - eta[4]*sinf(q[7]))*eta[4]*cosf(q[6])*cosf(q[7])/l + v[2])/(R*eta[4]*sinf(q[6])*sinf(q[7]));
    u[4] = v[4];
}
float full_2_JPTD_dynamic_lin_det_Kdd(const float q[], const float eta[]) {
    float det_Kdd = -1.0F/2.0F*powf(R, 3)*powf(eta[2], 2)*eta[4]*sinf(q[6])*sinf(q[7])*cosf(q[3])*powf(cosf(q[4]), 2)/l;
    return det_Kdd;
}
