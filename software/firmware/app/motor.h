#ifndef MOTOR_H
#define MOTOR_H

#include <stdint.h>

#include "stm32u5xx_hal.h"

typedef enum {
    MOTOR_STATE_IDLE,
    MOTOR_STATE_STARTUP_ALIGN1,
    MOTOR_STATE_STARTUP_ALIGN2,
    MOTOR_STATE_STARTUP_OPEN_LOOP,
    MOTOR_STATE_RUNNING,
    MOTOR_STATE_PANIC,
} motor_state_t;

typedef enum {
    MOTOR_PHASE_U,
    MOTOR_PHASE_V,
    MOTOR_PHASE_W,
} motor_phase_t;

typedef struct {
    float kp;
    float ki;
    volatile float process;
    volatile float setpoint;
    volatile float value;
    volatile float error_integral;
    volatile float error_prev;
} motor_pid_t;

typedef struct {
    TIM_HandleTypeDef *control_timer;
    TIM_HandleTypeDef *timebase_timer;
    motor_pid_t pid_pulse;
    motor_pid_t pid_commut;
    motor_state_t state;
    uint32_t state_start_time;
    uint32_t commut_task;
    uint32_t zc_task;
    volatile uint32_t zc_count;
    volatile uint8_t step;
    volatile float pulse;
    float vel;
    float vel_setpoint;
} motor_t;

void motor_init(motor_t *motor);
void motor_tick(motor_t *motor);
void motor_set_vel(motor_t *motor, const float vel);

void motor_commutation_callback(motor_t *motor, const TIM_HandleTypeDef *htim);
void motor_autoreload_callback(motor_t *motor, const TIM_HandleTypeDef *htim);
void motor_interrupt_callback(motor_t *motor, const uint16_t pin);

#endif
