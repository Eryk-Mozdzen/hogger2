#include "simplified_static_lin.h"


#include "robot_parameters.h"

void simplified_feedback_u(float u[5], const float K[25], const float q[9], const float h[5], const float hd[5], const float d_hd[5]) {
    
    u[0] = -K[0]*(d*cosf(q[2] + q[3]) - hd[0] + q[0]) - K[1]*(d*sinf(q[2] + q[3]) - hd[1] + q[1]) - K[2]*(-hd[2] + h[2]) - K[3]*(-hd[3] + h[3]) - K[4]*(-hd[4] + h[4]) + d_hd[0];
    u[1] = -K[5]*(d*cosf(q[2] + q[3]) - hd[0] + q[0]) - K[6]*(d*sinf(q[2] + q[3]) - hd[1] + q[1]) - K[7]*(-hd[2] + h[2]) - K[8]*(-hd[3] + h[3]) - K[9]*(-hd[4] + h[4]) + d_hd[1];
    u[2] = -K[10]*(d*cosf(q[2] + q[3]) - hd[0] + q[0]) - K[11]*(d*sinf(q[2] + q[3]) - hd[1] + q[1]) - K[12]*(-hd[2] + h[2]) - K[13]*(-hd[3] + h[3]) - K[14]*(-hd[4] + h[4]) + d_hd[2];
    u[3] = -K[15]*(d*cosf(q[2] + q[3]) - hd[0] + q[0]) - K[16]*(d*sinf(q[2] + q[3]) - hd[1] + q[1]) - K[17]*(-hd[2] + h[2]) - K[18]*(-hd[3] + h[3]) - K[19]*(-hd[4] + h[4]) + d_hd[3];
    u[4] = -K[20]*(d*cosf(q[2] + q[3]) - hd[0] + q[0]) - K[21]*(d*sinf(q[2] + q[3]) - hd[1] + q[1]) - K[22]*(-hd[2] + h[2]) - K[23]*(-hd[3] + h[3]) - K[24]*(-hd[4] + h[4]) + d_hd[4];
}
void simplified_static_lin_eta(float eta[5], const float u[5], const float q[9]) {
    
    eta[0] = u[0]*(-1.0F/2.0F*sinf(q[3] - q[5])*cosf(q[2] + q[3])/(l*sinf(q[5])) - sinf(q[2] + q[3])/d) + u[1]*(-1.0F/2.0F*sinf(q[2] + q[3])*sinf(q[3] - q[5])/(l*sinf(q[5])) + cosf(q[2] + q[3])/d);
    eta[1] = u[0]*cosf(q[2] + q[3])/q[7] + u[1]*sinf(q[2] + q[3])/q[7];
    eta[2] = u[2];
    eta[3] = u[3];
    eta[4] = u[4];
}
float simplified_static_lin_det_D(const float q[]) {
    float det_D = -d*q[7];
    return det_D;
}
