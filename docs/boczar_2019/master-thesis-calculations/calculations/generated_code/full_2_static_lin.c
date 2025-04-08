#include "full_2_static_lin.h"


#include "robot_parameters.h"

void full_2_feedback_u(float u[5], const float K[25], const float q[9], const float h[5], const float hd[5], const float d_hd[5]) {
    
    u[0] = -K[0]*(-hd[0] + h[0]) - K[1]*(-hd[1] + h[1]) - K[2]*(-hd[2] + h[2]) - K[3]*(-hd[3] + h[3]) - K[4]*(-hd[4] + h[4]) + d_hd[0];
    u[1] = -K[5]*(-hd[0] + h[0]) - K[6]*(-hd[1] + h[1]) - K[7]*(-hd[2] + h[2]) - K[8]*(-hd[3] + h[3]) - K[9]*(-hd[4] + h[4]) + d_hd[1];
    u[2] = -K[10]*(-hd[0] + h[0]) - K[11]*(-hd[1] + h[1]) - K[12]*(-hd[2] + h[2]) - K[13]*(-hd[3] + h[3]) - K[14]*(-hd[4] + h[4]) + d_hd[2];
    u[3] = -K[15]*(-hd[0] + h[0]) - K[16]*(-hd[1] + h[1]) - K[17]*(-hd[2] + h[2]) - K[18]*(-hd[3] + h[3]) - K[19]*(-hd[4] + h[4]) + d_hd[3];
    u[4] = -K[20]*(-hd[0] + h[0]) - K[21]*(-hd[1] + h[1]) - K[22]*(-hd[2] + h[2]) - K[23]*(-hd[3] + h[3]) - K[24]*(-hd[4] + h[4]) + d_hd[4];
}
void full_2_static_lin_eta(float eta[5], const float u[5], const float q[9]) {
    
    eta[0] = -u[3]*sinf(q[4]) + u[0]*sinf(q[2])/R - u[1]*cosf(q[2])/R;
    eta[1] = u[3]*cosf(q[4])*tanf(q[3]) + u[0]*cosf(q[2])/(R*cosf(q[3])) + u[1]*sinf(q[2])/(R*cosf(q[3]));
    eta[2] = u[3];
    eta[3] = u[4]*cosf(q[7])*tanf(q[6]) + 2*l*u[2]/(R*cosf(q[6])) + u[0]*cosf(q[2])/(R*cosf(q[6])) + u[1]*sinf(q[2])/(R*cosf(q[6]));
    eta[4] = u[4];
}
float full_2_static_lin_det_D(const float q[]) {
    float det_D = -1.0F/2.0F*powf(R, 3)*cosf(q[3])*cosf(q[6])/l;
    return det_D;
}
