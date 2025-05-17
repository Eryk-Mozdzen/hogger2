#include <stdbool.h>
#include <stdint.h>
#include <stm32h5xx_hal.h>

#include "actuate/motor.h"
#include "control/pid.h"

#define MOTOR_POLE_PAIRS 7
#define PI               3.141592653589f
#define VEL_THRESHOLD    50.f
#define VEL_PERIOD       20
#define VEL_FILTER       0.9f

#define ALIGN_TIME  200
#define ALIGN_PULSE 0.3f

#define OPEN_LOOP_PULSE       0.4f
#define OPEN_LOOP_RAMP_TIME   1000
#define OPEN_LOOP_RAMP_LAMBDA 1.f
#define OPEN_LOOP_RAMP_MIN    1000
#define OPEN_LOOP_RAMP_MAX    200000

#define CLOSED_LOOP_SWITCH 1000

// #define EXPERIMENT

#define CLAMP(val, min, max) (((val) > (max)) ? (max) : (((val) < (min)) ? (min) : (val)))

#ifndef EXPERIMENT
#define DIRECTION(val) ((val >= 0) ? MOTOR_DIRECTION_CW : MOTOR_DIRECTION_CCW)
#else
#define DIRECTION(val) MOTOR_DIRECTION_CW
#endif

static const motor_phase_t feedback_src_lookup_cw[6] = {
    MOTOR_PHASE_U, MOTOR_PHASE_W, MOTOR_PHASE_V, MOTOR_PHASE_U, MOTOR_PHASE_W, MOTOR_PHASE_V,
};

static const motor_phase_t feedback_src_lookup_ccw[6] = {
    MOTOR_PHASE_V, MOTOR_PHASE_U, MOTOR_PHASE_W, MOTOR_PHASE_V, MOTOR_PHASE_U, MOTOR_PHASE_W,
};

static const uint8_t feedback_dir_lookup[6] = {
    0, 1, 0, 1, 0, 1,
};

static const uint8_t filter_lookup[256] = {
    0, 1, 1, 2, 1, 2, 2, 3, 1, 2, 2, 3, 2, 3, 3, 4, 1, 2, 2, 3, 2, 3, 3, 4, 2, 3, 3, 4, 3, 4, 4, 5,
    1, 2, 2, 3, 2, 3, 3, 4, 2, 3, 3, 4, 3, 4, 4, 5, 2, 3, 3, 4, 3, 4, 4, 5, 3, 4, 4, 5, 4, 5, 5, 6,
    1, 2, 2, 3, 2, 3, 3, 4, 2, 3, 3, 4, 3, 4, 4, 5, 2, 3, 3, 4, 3, 4, 4, 5, 3, 4, 4, 5, 4, 5, 5, 6,
    2, 3, 3, 4, 3, 4, 4, 5, 3, 4, 4, 5, 4, 5, 5, 6, 3, 4, 4, 5, 4, 5, 5, 6, 4, 5, 5, 6, 5, 6, 6, 7,
    1, 2, 2, 3, 2, 3, 3, 4, 2, 3, 3, 4, 3, 4, 4, 5, 2, 3, 3, 4, 3, 4, 4, 5, 3, 4, 4, 5, 4, 5, 5, 6,
    2, 3, 3, 4, 3, 4, 4, 5, 3, 4, 4, 5, 4, 5, 5, 6, 3, 4, 4, 5, 4, 5, 5, 6, 4, 5, 5, 6, 5, 6, 6, 7,
    2, 3, 3, 4, 3, 4, 4, 5, 3, 4, 4, 5, 4, 5, 5, 6, 3, 4, 4, 5, 4, 5, 5, 6, 4, 5, 5, 6, 5, 6, 6, 7,
    3, 4, 4, 5, 4, 5, 5, 6, 4, 5, 5, 6, 5, 6, 6, 7, 4, 5, 5, 6, 5, 6, 6, 7, 5, 6, 6, 7, 6, 7, 7, 8,
};

static void config_pwm(const motor_t *motor, const uint32_t channel) {
    const uint32_t pulse = motor->pulse * motor->control_timer->Instance->ARR;

    HAL_TIM_OC_Stop(motor->control_timer, channel);
    HAL_TIM_PWM_Stop(motor->control_timer, channel);
    HAL_TIMEx_OCN_Stop(motor->control_timer, channel);
    HAL_TIMEx_PWMN_Stop(motor->control_timer, channel);

    const TIM_OC_InitTypeDef config = {
        .OCMode = TIM_OCMODE_PWM1,
        .Pulse = pulse,
        .OCPolarity = TIM_OCPOLARITY_HIGH,
        .OCNPolarity = TIM_OCNPOLARITY_HIGH,
        .OCFastMode = TIM_OCFAST_DISABLE,
        .OCIdleState = TIM_OCIDLESTATE_RESET,
        .OCNIdleState = TIM_OCNIDLESTATE_RESET,
    };
    HAL_TIM_PWM_ConfigChannel(motor->control_timer, &config, channel);

    HAL_TIM_PWM_Start(motor->control_timer, channel);
    HAL_TIMEx_PWMN_Start(motor->control_timer, channel);
}

static void config_oc(const motor_t *motor, const uint32_t channel, const uint32_t mode) {
    const uint32_t pulse = motor->pulse * motor->control_timer->Instance->ARR;

    HAL_TIM_OC_Stop(motor->control_timer, channel);
    HAL_TIM_PWM_Stop(motor->control_timer, channel);
    HAL_TIMEx_OCN_Stop(motor->control_timer, channel);
    HAL_TIMEx_PWMN_Stop(motor->control_timer, channel);

    const TIM_OC_InitTypeDef config = {
        .OCMode = mode,
        .Pulse = pulse,
        .OCPolarity = TIM_OCPOLARITY_HIGH,
        .OCNPolarity = TIM_OCNPOLARITY_HIGH,
        .OCFastMode = TIM_OCFAST_DISABLE,
        .OCIdleState = TIM_OCIDLESTATE_RESET,
        .OCNIdleState = TIM_OCNIDLESTATE_RESET,
    };
    HAL_TIM_OC_ConfigChannel(motor->control_timer, &config, channel);

    HAL_TIM_OC_Stop(motor->control_timer, channel);
    HAL_TIMEx_OCN_Start(motor->control_timer, channel);
}

static void shutdown(const motor_t *motor) {
    HAL_TIM_Base_Stop(motor->commut_timer);
    HAL_TIM_Base_Stop_IT(motor->control_timer);
    HAL_ADCEx_InjectedStop_IT(motor->bemf_adc);

    HAL_TIM_OC_Stop(motor->control_timer, TIM_CHANNEL_1);
    HAL_TIM_OC_Stop(motor->control_timer, TIM_CHANNEL_2);
    HAL_TIM_OC_Stop(motor->control_timer, TIM_CHANNEL_3);

    HAL_TIMEx_OCN_Stop(motor->control_timer, TIM_CHANNEL_1);
    HAL_TIMEx_OCN_Stop(motor->control_timer, TIM_CHANNEL_2);
    HAL_TIMEx_OCN_Stop(motor->control_timer, TIM_CHANNEL_3);

    HAL_TIM_PWM_Stop(motor->control_timer, TIM_CHANNEL_1);
    HAL_TIM_PWM_Stop(motor->control_timer, TIM_CHANNEL_2);
    HAL_TIM_PWM_Stop(motor->control_timer, TIM_CHANNEL_3);

    HAL_TIMEx_PWMN_Stop(motor->control_timer, TIM_CHANNEL_1);
    HAL_TIMEx_PWMN_Stop(motor->control_timer, TIM_CHANNEL_2);
    HAL_TIMEx_PWMN_Stop(motor->control_timer, TIM_CHANNEL_3);
}

static void state_change(motor_t *motor, const motor_state_t new) {
    const uint32_t time = HAL_GetTick();

    motor->state = new;
    motor->state_start_time = time;
}

static bool software_timer(uint32_t *prev, const uint32_t time, const uint32_t period) {
    if((time - *prev) >= period) {
        *prev = time;
        return true;
    }

    return false;
}

void motor_init(motor_t *motor) {
    motor->state = MOTOR_STATE_IDLE;
    motor->step = 0;
    motor->pulse = 0;
    motor->vel = 0;
    motor->vel_setpoint = 0;

    shutdown(motor);
}

void motor_tick(motor_t *motor) {
    const uint32_t time = HAL_GetTick() - motor->state_start_time;

    if(software_timer(&motor->vel_task, time, VEL_PERIOD)) {
        const int32_t count = motor->zc_count;
        motor->zc_count = 0;

        const float velocity = (2.f * PI * count) / (6 * MOTOR_POLE_PAIRS * VEL_PERIOD * 0.001f);
        motor->vel = VEL_FILTER * motor->vel + (1.f - VEL_FILTER) * velocity;
    }

    switch(motor->state) {
        case MOTOR_STATE_IDLE: {
            if(fabs(motor->vel_setpoint) > VEL_THRESHOLD) {
                motor->direction = DIRECTION(motor->vel_setpoint);
                motor->step = 0;
                motor->pulse = ALIGN_PULSE;

                state_change(motor, MOTOR_STATE_STARTUP_ALIGN1);

                pid_reset(&motor->pid);
                __HAL_TIM_SET_COUNTER(motor->commut_timer, 0);
                __HAL_TIM_SET_AUTORELOAD(motor->commut_timer, 1000);
                HAL_TIM_Base_Start(motor->commut_timer);
                HAL_TIM_Base_Start_IT(motor->control_timer);
                HAL_TIMEx_ConfigCommutEvent_IT(motor->control_timer, motor->control_timer_itr,
                                               TIM_COMMUTATION_TRGI);
                HAL_ADCEx_InjectedStart_IT(motor->bemf_adc);
            }
        } break;
        case MOTOR_STATE_STARTUP_ALIGN1: {
            if(fabs(motor->vel_setpoint) < VEL_THRESHOLD) {
                state_change(motor, MOTOR_STATE_PANIC);
            }

            if(DIRECTION(motor->vel_setpoint) != motor->direction) {
                state_change(motor, MOTOR_STATE_PANIC);
            }

            if(time >= ALIGN_TIME) {
                if(motor->direction == MOTOR_DIRECTION_CW) {
                    motor->step = 1;
                } else {
                    motor->step = 5;
                }
                motor->pulse = ALIGN_PULSE;

                state_change(motor, MOTOR_STATE_STARTUP_ALIGN2);
            }
        } break;
        case MOTOR_STATE_STARTUP_ALIGN2: {
            if(fabs(motor->vel_setpoint) < VEL_THRESHOLD) {
                state_change(motor, MOTOR_STATE_PANIC);
            }

            if(DIRECTION(motor->vel_setpoint) != motor->direction) {
                state_change(motor, MOTOR_STATE_PANIC);
            }

            if(time >= ALIGN_TIME) {
                motor->pulse = OPEN_LOOP_PULSE;
                __HAL_TIM_SET_AUTORELOAD(motor->commut_timer, OPEN_LOOP_RAMP_MAX);

                state_change(motor, MOTOR_STATE_STARTUP_OPEN_LOOP);

                motor->ramp_task = 0;
                motor->zc_count = 0;
                motor->switch_over = 0;
            }
        } break;
        case MOTOR_STATE_STARTUP_OPEN_LOOP: {
            if(software_timer(&motor->ramp_task, time, 1)) {
                const float t = time * 0.001f;
                const float f = 1.f - expf(-OPEN_LOOP_RAMP_LAMBDA * t);
                const uint32_t T =
                    CLAMP(OPEN_LOOP_RAMP_MIN / f, OPEN_LOOP_RAMP_MIN, OPEN_LOOP_RAMP_MAX);

                __HAL_TIM_SET_AUTORELOAD(motor->commut_timer, T);
                if(__HAL_TIM_GET_COUNTER(motor->commut_timer) >= T) {
                    __HAL_TIM_SET_COUNTER(motor->commut_timer, T - 1);
                }
            }

            if(fabs(motor->vel_setpoint) < VEL_THRESHOLD) {
                state_change(motor, MOTOR_STATE_PANIC);
            }

            if(DIRECTION(motor->vel_setpoint) != motor->direction) {
                state_change(motor, MOTOR_STATE_PANIC);
            }

            if(time >= OPEN_LOOP_RAMP_TIME) {
                if(fabs(motor->vel) > VEL_THRESHOLD) {
                    state_change(motor, MOTOR_STATE_RUNNING);
                } else {
                    state_change(motor, MOTOR_STATE_PANIC);
                }
            }
        } break;
        case MOTOR_STATE_RUNNING: {
            if(time <= CLOSED_LOOP_SWITCH) {
                motor->switch_over = ((float)time) / ((float)CLOSED_LOOP_SWITCH);
            } else {
                motor->switch_over = 1.f;
            }

            if(fabs(motor->vel_setpoint) < VEL_THRESHOLD) {
                state_change(motor, MOTOR_STATE_PANIC);
            }

            if(DIRECTION(motor->vel_setpoint) != motor->direction) {
                state_change(motor, MOTOR_STATE_PANIC);
            }

            if(fabs(motor->vel) < VEL_THRESHOLD) {
                state_change(motor, MOTOR_STATE_PANIC);
            }
        } break;
        case MOTOR_STATE_PANIC: {
            shutdown(motor);

            state_change(motor, MOTOR_STATE_IDLE);

            motor->pulse = 0;
            motor->vel_setpoint = 0;
        } break;
    }
}

void motor_commutation_callback(motor_t *motor, const TIM_HandleTypeDef *htim) {
    if(htim != motor->control_timer) {
        return;
    }

    if(motor->state == MOTOR_STATE_RUNNING) {
#ifndef EXPERIMENT
        motor->pulse = pid_calculate(&motor->pid, fabs(motor->vel_setpoint), fabs(motor->vel));
#else
        motor->pulse = ((HAL_GetTick() - motor->state_start_time) > 5000) ? 0.4f : 0.3f;
#endif
    }

    if((motor->state == MOTOR_STATE_STARTUP_OPEN_LOOP) || (motor->state == MOTOR_STATE_RUNNING)) {
        if(motor->direction == MOTOR_DIRECTION_CW) {
            motor->step = (motor->step + 6 + 1) % 6;
        } else {
            motor->step = (motor->step + 6 - 1) % 6;
        }
    }

    switch(motor->step) {
        case 0: {
            config_pwm(motor, TIM_CHANNEL_1);
            config_oc(motor, TIM_CHANNEL_2, TIM_OCMODE_FORCED_ACTIVE);
            config_oc(motor, TIM_CHANNEL_3, TIM_OCMODE_FORCED_INACTIVE);
        } break;
        case 1: {
            config_pwm(motor, TIM_CHANNEL_1);
            config_oc(motor, TIM_CHANNEL_2, TIM_OCMODE_FORCED_INACTIVE);
            config_oc(motor, TIM_CHANNEL_3, TIM_OCMODE_FORCED_ACTIVE);
        } break;
        case 2: {
            config_oc(motor, TIM_CHANNEL_1, TIM_OCMODE_FORCED_INACTIVE);
            config_pwm(motor, TIM_CHANNEL_2);
            config_oc(motor, TIM_CHANNEL_3, TIM_OCMODE_FORCED_ACTIVE);
        } break;
        case 3: {
            config_oc(motor, TIM_CHANNEL_1, TIM_OCMODE_FORCED_ACTIVE);
            config_pwm(motor, TIM_CHANNEL_2);
            config_oc(motor, TIM_CHANNEL_3, TIM_OCMODE_FORCED_INACTIVE);
        } break;
        case 4: {
            config_oc(motor, TIM_CHANNEL_1, TIM_OCMODE_FORCED_ACTIVE);
            config_oc(motor, TIM_CHANNEL_2, TIM_OCMODE_FORCED_INACTIVE);
            config_pwm(motor, TIM_CHANNEL_3);
        } break;
        case 5: {
            config_oc(motor, TIM_CHANNEL_1, TIM_OCMODE_FORCED_INACTIVE);
            config_oc(motor, TIM_CHANNEL_2, TIM_OCMODE_FORCED_ACTIVE);
            config_pwm(motor, TIM_CHANNEL_3);
        } break;
    }

    motor->zc_filter = 0;
    motor->zc_occur = 0;
}

void motor_sample_callback(motor_t *motor, const ADC_HandleTypeDef *hadc) {
    const uint32_t counter = __HAL_TIM_GET_COUNTER(motor->commut_timer);
    const uint32_t autoreload = __HAL_TIM_GET_AUTORELOAD(motor->commut_timer);

    if((hadc != motor->bemf_adc) || motor->zc_occur) {
        return;
    }

    const uint32_t bemf[3] = {
        HAL_ADCEx_InjectedGetValue(motor->bemf_adc, ADC_INJECTED_RANK_1),
        HAL_ADCEx_InjectedGetValue(motor->bemf_adc, ADC_INJECTED_RANK_2),
        HAL_ADCEx_InjectedGetValue(motor->bemf_adc, ADC_INJECTED_RANK_3),
    };

    const uint32_t neutral = (bemf[MOTOR_PHASE_U] + bemf[MOTOR_PHASE_V] + bemf[MOTOR_PHASE_W]) / 3;

    if(motor->direction == MOTOR_DIRECTION_CW) {
        const uint8_t state = (bemf[feedback_src_lookup_cw[motor->step]] > neutral);

        motor->zc_filter <<= 1;
        motor->zc_filter |= (state ^ feedback_dir_lookup[motor->step]);
    } else {
        const uint8_t state = (bemf[feedback_src_lookup_ccw[motor->step]] < neutral);

        motor->zc_filter <<= 1;
        motor->zc_filter |= (state ^ feedback_dir_lookup[motor->step]);
    }

    const float alpha = 0.05f;
    const uint8_t ones = 4;
    if(filter_lookup[motor->zc_filter] >= ones) {
        motor->zc_occur = 1;

        if(motor->direction == MOTOR_DIRECTION_CW) {
            motor->zc_count++;
        } else {
            motor->zc_count--;
        }

        if(motor->state == MOTOR_STATE_RUNNING) {
            const uint32_t zc_time = counter - ((ones / 2) * 20);

            uint32_t period = alpha * (2 * zc_time) + (1.f - alpha) * autoreload;

            if(motor->switch_over < 1.f) {
                const uint32_t time = HAL_GetTick() - motor->state_start_time + OPEN_LOOP_RAMP_TIME;
                const float t = time * 0.001f;
                const float f = 1.f - expf(-OPEN_LOOP_RAMP_LAMBDA * t);
                const uint32_t T =
                    CLAMP(OPEN_LOOP_RAMP_MIN / f, OPEN_LOOP_RAMP_MIN, OPEN_LOOP_RAMP_MAX);

                period = (1.f - motor->switch_over) * T + motor->switch_over * period;
            }

            __HAL_TIM_SET_AUTORELOAD(motor->commut_timer, period);
        }
    }
}
