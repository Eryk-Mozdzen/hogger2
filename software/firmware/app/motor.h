#ifndef MOTOR_H
#define MOTOR_H

#include <stdint.h>
#include <stdbool.h>

#include "stm32u5xx_hal.h"

typedef enum {
    MOTOR_STATE_IDLE,
    MOTOR_STATE_STARTUP_ALIGN1,
    MOTOR_STATE_STARTUP_ALIGN2,
    MOTOR_STATE_STARTUP_OPEN_LOOP,
    MOTOR_STATE_RUNNING,
    MOTOR_STATE_PANIC,
} motor_state_t;

typedef struct {
    TIM_HandleTypeDef *control_timer;
    TIM_HandleTypeDef *startup_timer;
    motor_state_t state;
    uint32_t state_start_time;
    uint32_t tick_last_time;
    uint8_t step;
    float pulse;

    float vel;
    float vel_setpoint;
} motor_t;

void motor_init(motor_t *motor);
void motor_tick(motor_t *motor);
void motor_set_vel(motor_t *motor, const float vel);

void motor_commut_callback(motor_t *motor, TIM_HandleTypeDef *htim);

#endif
