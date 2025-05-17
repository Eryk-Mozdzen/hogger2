#ifndef MOTOR_H
#define MOTOR_H

#include <stdint.h>
#include <stm32h5xx_hal.h>

#include "control/pid.h"

#define MOTOR_MIN_PULSE       0.15f
#define MOTOR_MAX_PULSE       0.8f
#define MOTOR_PID(kp, ki, kd) PID_INIT((kp), (ki), (kd), MOTOR_MIN_PULSE, MOTOR_MAX_PULSE)

typedef enum {
    MOTOR_STATE_IDLE,
    MOTOR_STATE_STARTUP_ALIGN1,
    MOTOR_STATE_STARTUP_ALIGN2,
    MOTOR_STATE_STARTUP_OPEN_LOOP,
    MOTOR_STATE_RUNNING,
    MOTOR_STATE_PANIC,
} motor_state_t;

typedef enum {
    MOTOR_DIRECTION_CW,
    MOTOR_DIRECTION_CCW,
} motor_direction_t;

typedef enum {
    MOTOR_PHASE_U,
    MOTOR_PHASE_V,
    MOTOR_PHASE_W,
} motor_phase_t;

typedef struct {
    TIM_HandleTypeDef *control_timer;
    TIM_HandleTypeDef *commut_timer;
    uint32_t control_timer_itr;
    ADC_HandleTypeDef *bemf_adc;
    pid_t pid;
    motor_state_t state;
    motor_direction_t direction;
    uint32_t state_start_time;
    uint32_t ramp_task;
    uint32_t vel_task;
    float switch_over;
    volatile float pulse;
    volatile uint8_t step;
    volatile uint8_t zc_filter;
    volatile uint8_t zc_occur;
    volatile int32_t zc_count;
    float vel;
    float vel_setpoint;
} motor_t;

void motor_init(motor_t *motor);
void motor_tick(motor_t *motor);

void motor_commutation_callback(motor_t *motor, const TIM_HandleTypeDef *htim);
void motor_sample_callback(motor_t *motor, const ADC_HandleTypeDef *hadc);

#endif
