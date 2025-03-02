#include <cstring>
#include <stm32u5xx_hal.h>

#include "actuate/BLDC.hpp"
#include "com/TelemetrySource.hpp"
#include "freertos/Task.hpp"

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

#define CLOSED_LOOP_SWITCH    1000
#define CLOSED_LOOP_PULSE_MIN 0.15f
#define CLOSED_LOOP_PULSE_MAX 0.8f
#define CLOSED_LOOP_KP        0.001f
#define CLOSED_LOOP_KI        0.000001f

#define CLAMP(val, min, max) (((val) > (max)) ? (max) : (((val) < (min)) ? (min) : (val)));

BLDC::PID::PID() : kp{CLOSED_LOOP_KP}, ki{CLOSED_LOOP_KI} {
    process = 0;
    setpoint = 0;
    value = 0;
    dt = 1;
    error_integral = 0;
    error_prev = 0;
}

void BLDC::PID::reset() {
    process = 0;
    setpoint = 0;
    value = 0;
    dt = 1;
    error_integral = 0;
    error_prev = 0;
}

void BLDC::PID::calculate() {
    const float error = setpoint - process;

    error_integral += 0.5f * dt * (error_prev + error);

    value = kp * error + ki * error_integral;

    error_prev = error;
}

BLDC::BLDC(const char *name, const Config &config) : name{name}, config{config} {
    state = PANIC;
    step = 0;
    pulse = 0;
    vel = 0;
    vel_setpoint = 0;
}

void BLDC::configPWM(const uint32_t channel) {
    const uint32_t compare = pulse * config.control_timer->Instance->ARR;

    HAL_TIM_OC_Stop(config.control_timer, channel);
    HAL_TIM_PWM_Stop(config.control_timer, channel);
    HAL_TIMEx_OCN_Stop(config.control_timer, channel);
    HAL_TIMEx_PWMN_Stop(config.control_timer, channel);

    TIM_OC_InitTypeDef cfg;
    cfg.OCMode = TIM_OCMODE_PWM1;
    cfg.Pulse = compare;
    cfg.OCPolarity = TIM_OCPOLARITY_HIGH;
    cfg.OCNPolarity = TIM_OCNPOLARITY_HIGH;
    cfg.OCFastMode = TIM_OCFAST_DISABLE;
    cfg.OCIdleState = TIM_OCIDLESTATE_RESET;
    cfg.OCNIdleState = TIM_OCNIDLESTATE_RESET;
    HAL_TIM_PWM_ConfigChannel(config.control_timer, &cfg, channel);

    HAL_TIM_PWM_Start(config.control_timer, channel);
    HAL_TIMEx_PWMN_Start(config.control_timer, channel);
}

void BLDC::configOC(const uint32_t channel, const uint32_t mode) {
    const uint32_t compare = pulse * config.control_timer->Instance->ARR;

    HAL_TIM_OC_Stop(config.control_timer, channel);
    HAL_TIM_PWM_Stop(config.control_timer, channel);
    HAL_TIMEx_OCN_Stop(config.control_timer, channel);
    HAL_TIMEx_PWMN_Stop(config.control_timer, channel);

    TIM_OC_InitTypeDef cfg;
    cfg.OCMode = mode;
    cfg.Pulse = compare;
    cfg.OCPolarity = TIM_OCPOLARITY_HIGH;
    cfg.OCNPolarity = TIM_OCNPOLARITY_HIGH;
    cfg.OCFastMode = TIM_OCFAST_DISABLE;
    cfg.OCIdleState = TIM_OCIDLESTATE_RESET;
    cfg.OCNIdleState = TIM_OCNIDLESTATE_RESET;
    HAL_TIM_OC_ConfigChannel(config.control_timer, &cfg, channel);

    HAL_TIM_OC_Stop(config.control_timer, channel);
    HAL_TIMEx_OCN_Start(config.control_timer, channel);
}

void BLDC::shutdown() {
    HAL_TIM_Base_Stop(config.commut_timer);
    HAL_TIM_Base_Stop_IT(config.control_timer);

    HAL_TIM_OC_Stop(config.control_timer, TIM_CHANNEL_1);
    HAL_TIM_OC_Stop(config.control_timer, TIM_CHANNEL_2);
    HAL_TIM_OC_Stop(config.control_timer, TIM_CHANNEL_3);

    HAL_TIMEx_OCN_Stop(config.control_timer, TIM_CHANNEL_1);
    HAL_TIMEx_OCN_Stop(config.control_timer, TIM_CHANNEL_2);
    HAL_TIMEx_OCN_Stop(config.control_timer, TIM_CHANNEL_3);

    HAL_TIM_PWM_Stop(config.control_timer, TIM_CHANNEL_1);
    HAL_TIM_PWM_Stop(config.control_timer, TIM_CHANNEL_2);
    HAL_TIM_PWM_Stop(config.control_timer, TIM_CHANNEL_3);

    HAL_TIMEx_PWMN_Stop(config.control_timer, TIM_CHANNEL_1);
    HAL_TIMEx_PWMN_Stop(config.control_timer, TIM_CHANNEL_2);
    HAL_TIMEx_PWMN_Stop(config.control_timer, TIM_CHANNEL_3);
}

void BLDC::moveTo(const State next) {
    state = next;
    state_start_time = system_time;
}

void BLDC::serialize(cmp_ctx_t *cmp) const {
    const char *stateStr[] = {
        "idle", "startup_align_1", "startup_align_2", "startup_ramp", "running", "panic",
    };

    cmp_write_map(cmp, 4);
    cmp_write_str(cmp, "state", 5);
    cmp_write_str(cmp, stateStr[state], strlen(stateStr[state]));
    cmp_write_str(cmp, "vel_ref", 7);
    cmp_write_float(cmp, vel_setpoint);
    cmp_write_str(cmp, "vel", 3);
    cmp_write_float(cmp, vel);
    cmp_write_str(cmp, "load", 4);
    cmp_write_float(cmp, pulse);
}

const char *BLDC::getName() const {
    return name;
}

void BLDC::setVelocity(const float velocity) {
    vel_setpoint = velocity;
}

float BLDC::getVelocity() const {
    return vel;
}

bool BLDC::softwareTimer(uint32_t *prev, const uint32_t time, const uint32_t period) {
    if((time - *prev) >= period) {
        *prev = time;
        return true;
    }

    return false;
}

void BLDC::tick() {
    system_time = xTaskGetTickCount();
    const uint32_t time = system_time - state_start_time;

    if(softwareTimer(&vel_task, time, VEL_PERIOD)) {
        const uint32_t count = zc_count;
        zc_count = 0;

        const float velocity = (2.f * PI * count) / (6 * MOTOR_POLE_PAIRS * VEL_PERIOD * 0.001f);
        vel = VEL_FILTER * vel + (1.f - VEL_FILTER) * velocity;
    }

    switch(state) {
        case IDLE: {
            if(vel_setpoint > VEL_THRESHOLD) {
                step = 0;
                pulse = ALIGN_PULSE;

                moveTo(STARTUP_ALIGN1);

                pid.reset();
                __HAL_TIM_SET_COUNTER(config.commut_timer, 0);
                __HAL_TIM_SET_AUTORELOAD(config.commut_timer, 1000);
                HAL_TIM_Base_Start(config.commut_timer);
                HAL_TIM_Base_Start_IT(config.control_timer);
                HAL_TIMEx_ConfigCommutEvent_IT(config.control_timer, config.control_timer_itr,
                                               TIM_COMMUTATION_TRGI);
            }
        } break;
        case STARTUP_ALIGN1: {
            if(vel_setpoint < VEL_THRESHOLD) {
                moveTo(PANIC);
            }

            if(time >= ALIGN_TIME) {
                step = 1;
                pulse = ALIGN_PULSE;

                moveTo(STARTUP_ALIGN2);
            }
        } break;
        case STARTUP_ALIGN2: {
            if(vel_setpoint < VEL_THRESHOLD) {
                moveTo(PANIC);
            }

            if(time >= ALIGN_TIME) {
                pulse = OPEN_LOOP_PULSE;
                __HAL_TIM_SET_AUTORELOAD(config.commut_timer, OPEN_LOOP_RAMP_MAX);

                moveTo(STARTUP_RAMP);

                ramp_task = 0;
                zc_count = 0;
                switch_over = 0;
            }
        } break;
        case STARTUP_RAMP: {
            if(softwareTimer(&ramp_task, time, 1)) {
                const float t = time * 0.001f;
                const float f = 1.f - expf(-OPEN_LOOP_RAMP_LAMBDA * t);
                const uint32_t T =
                    CLAMP(OPEN_LOOP_RAMP_MIN / f, OPEN_LOOP_RAMP_MIN, OPEN_LOOP_RAMP_MAX);

                __HAL_TIM_SET_AUTORELOAD(config.commut_timer, T);
                if(__HAL_TIM_GET_COUNTER(config.commut_timer) >= T) {
                    __HAL_TIM_SET_COUNTER(config.commut_timer, T - 1);
                }
            }

            if(vel_setpoint < VEL_THRESHOLD) {
                moveTo(PANIC);
            }

            if(time >= OPEN_LOOP_RAMP_TIME) {
                if(vel > VEL_THRESHOLD) {
                    moveTo(RUNNING);
                } else {
                    moveTo(PANIC);
                }
            }
        } break;
        case RUNNING: {
            if(time <= CLOSED_LOOP_SWITCH) {
                switch_over = ((float) time) / ((float) CLOSED_LOOP_SWITCH);
            } else {
                switch_over = 1.f;
            }

            if(vel_setpoint < VEL_THRESHOLD) {
                moveTo(PANIC);
            }

            if(vel < VEL_THRESHOLD) {
                moveTo(PANIC);
            }
        } break;
        case PANIC: {
            shutdown();

            pulse = 0;
            vel_setpoint = 0;

            moveTo(IDLE);
        } break;
    }
}

void BLDC::commutationISR(const TIM_HandleTypeDef *htim) {
    if(htim != config.control_timer) {
        return;
    }

    if(state == RUNNING) {
        pid.process = vel;
        pid.setpoint = vel_setpoint;
        pid.dt = __HAL_TIM_GET_AUTORELOAD(config.commut_timer) * 0.001f;
        pid.calculate();
        pulse = CLAMP(OPEN_LOOP_PULSE + switch_over * pid.value, CLOSED_LOOP_PULSE_MIN,
                      CLOSED_LOOP_PULSE_MAX);
    }

    if((state == STARTUP_RAMP) || (state == RUNNING)) {
        step++;
        step %= 6;
    }

    switch(step) {
        case 0: {
            configPWM(TIM_CHANNEL_1);
            configOC(TIM_CHANNEL_2, TIM_OCMODE_FORCED_ACTIVE);
            configOC(TIM_CHANNEL_3, TIM_OCMODE_FORCED_INACTIVE);
        } break;
        case 1: {
            configPWM(TIM_CHANNEL_1);
            configOC(TIM_CHANNEL_2, TIM_OCMODE_FORCED_INACTIVE);
            configOC(TIM_CHANNEL_3, TIM_OCMODE_FORCED_ACTIVE);
        } break;
        case 2: {
            configOC(TIM_CHANNEL_1, TIM_OCMODE_FORCED_INACTIVE);
            configPWM(TIM_CHANNEL_2);
            configOC(TIM_CHANNEL_3, TIM_OCMODE_FORCED_ACTIVE);
        } break;
        case 3: {
            configOC(TIM_CHANNEL_1, TIM_OCMODE_FORCED_ACTIVE);
            configPWM(TIM_CHANNEL_2);
            configOC(TIM_CHANNEL_3, TIM_OCMODE_FORCED_INACTIVE);
        } break;
        case 4: {
            configOC(TIM_CHANNEL_1, TIM_OCMODE_FORCED_ACTIVE);
            configOC(TIM_CHANNEL_2, TIM_OCMODE_FORCED_INACTIVE);
            configPWM(TIM_CHANNEL_3);
        } break;
        case 5: {
            configOC(TIM_CHANNEL_1, TIM_OCMODE_FORCED_INACTIVE);
            configOC(TIM_CHANNEL_2, TIM_OCMODE_FORCED_ACTIVE);
            configPWM(TIM_CHANNEL_3);
        } break;
    }

    zc_filter = 0;
    zc_occur = 0;
}

void BLDC::sampleISR(const TIM_HandleTypeDef *htim) {
    const uint32_t counter = __HAL_TIM_GET_COUNTER(config.commut_timer);
    const uint32_t autoreload = __HAL_TIM_GET_AUTORELOAD(config.commut_timer);

    if((htim != config.control_timer) || zc_occur) {
        return;
    }

    const BEMF &bemf = config.bemf[feedback_src_lookup[step]];
    const uint8_t signal = HAL_GPIO_ReadPin(bemf.port, bemf.pin);

    zc_filter <<= 1;
    zc_filter |= (signal ^ feedback_dir_lookup[step]);

    if(filter_lookup[zc_filter] >= 4) {
        zc_occur = 1;
        zc_count++;

        if(state == RUNNING) {
            uint32_t period = 0.2f * 2 * counter + 0.8f * autoreload;

            if(switch_over < 1.f) {
                const uint32_t time = system_time - state_start_time + OPEN_LOOP_RAMP_TIME;
                const float t = time * 0.001f;
                const float f = 1.f - expf(-OPEN_LOOP_RAMP_LAMBDA * t);
                const uint32_t T =
                    CLAMP(OPEN_LOOP_RAMP_MIN / f, OPEN_LOOP_RAMP_MIN, OPEN_LOOP_RAMP_MAX);

                period = (1.f - switch_over) * T + switch_over * period;
            }

            __HAL_TIM_SET_AUTORELOAD(config.commut_timer, period);
        }
    }
}
