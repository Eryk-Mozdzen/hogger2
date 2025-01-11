#ifndef ROBOT_H
#define ROBOT_H

#include "motor.h"
#include "imu.h"
#include "flow.h"

typedef enum {
    ROBOT_STATE_STOPPED,
    ROBOT_STATE_MANUAL,
    ROBOT_STATE_AUTONOMOUS,
} robot_state_t;

typedef struct {
    robot_state_t state;
    uint32_t time;
    float supply;
    struct {
        motor_t *motor;
        float servo_x;
        float servo_y;
    } hog[2];
    imu_t *imu;
    flow_t *flow;
} robot_t;

#endif
