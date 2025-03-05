#ifndef MOTOR_H
#define MOTOR_H

#include <stdint.h>
#include <stm32u5xx_hal.h>

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
    volatile float dt;
} motor_pid_t;

typedef struct {
    uint16_t pin;
    GPIO_TypeDef *port;
} motor_bemf_t;

typedef struct {
    TIM_HandleTypeDef *control_timer;
    TIM_HandleTypeDef *commut_timer;
    uint32_t control_timer_itr;
    motor_bemf_t bemf[3];
    motor_pid_t pid;
    motor_state_t state;
    uint32_t state_start_time;
    uint32_t ramp_task;
    uint32_t vel_task;
    float switch_over;
    volatile float pulse;
    volatile uint8_t step;
    volatile uint8_t zc_filter;
    volatile uint8_t zc_occur;
    volatile uint32_t zc_count;
    float vel;
    float vel_setpoint;
} motor_t;

void motor_init(motor_t *motor);
void motor_tick(motor_t *motor);

void motor_commutation_callback(motor_t *motor, const TIM_HandleTypeDef *htim);
void motor_sample_callback(motor_t *motor, const TIM_HandleTypeDef *htim);

#endif
