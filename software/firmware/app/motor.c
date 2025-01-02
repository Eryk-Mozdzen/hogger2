#include <stdint.h>
#include <stdbool.h>

#include "stm32u5xx_hal.h"
#include "motor.h"

#define MOTOR_POLE_PAIRS		7
#define PI						3.141592653589f
#define VEL_THRESHOLD			50.f
#define VEL_PERIOD				20
#define VEL_FILTER				0.9f

#define ALIGN_TIME              200
#define ALIGN_PULSE             0.3f

#define OPEN_LOOP_PULSE			0.2f
#define OPEN_LOOP_RAMP_TIME		2000
#define OPEN_LOOP_RAMP_LAMBDA	3.f
#define OPEN_LOOP_RAMP_MIN		2000
#define OPEN_LOOP_RAMP_MAX		200000

#define CLOSED_LOOP_PULSE_MIN   0.1f
#define CLOSED_LOOP_PULSE_MAX   0.3f
#define CLOSED_LOOP_KP          0.001f
#define CLOSED_LOOP_KI          0.000001f

#define CLAMP(val, min, max)	(((val)>(max)) ? (max) : (((val)<(min)) ? (min) : (val)));

static void config_pwm(const motor_t *motor, const uint32_t channel) {
    const uint32_t pulse = motor->pulse*motor->control_timer->Instance->ARR;

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
	const uint32_t pulse = motor->pulse*motor->control_timer->Instance->ARR;

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
	if((time - *prev)>=period) {
		*prev = time;
		return true;
	}

	return false;
}

static void pid_init(motor_pid_t *pid) {
	pid->process = 0;
	pid->setpoint = 0;
	pid->value = 0;

	pid->error_integral = 0;
	pid->error_prev = 0;
}

static void pid_calculate(motor_pid_t *pid) {
	const float error = pid->setpoint - pid->process;

	pid->error_integral +=0.5f*(pid->error_prev + error);

	pid->value = pid->kp*error + pid->ki*pid->error_integral;

	pid->error_prev = error;
}

void motor_init(motor_t *motor) {
    motor->state = MOTOR_STATE_IDLE;
    motor->step = 0;
	motor->pulse = 0;
	motor->vel = 0;
	motor->vel_setpoint = 0;

	motor->pid.kp = CLOSED_LOOP_KP;
	motor->pid.ki = CLOSED_LOOP_KI;

	shutdown(motor);
}

void motor_tick(motor_t *motor) {
	const uint32_t time = HAL_GetTick() - motor->state_start_time;

    switch(motor->state) {
        case MOTOR_STATE_IDLE: {
			if(motor->vel_setpoint>VEL_THRESHOLD) {
				motor->step = 0;
				motor->pulse = ALIGN_PULSE;

				state_change(motor, MOTOR_STATE_STARTUP_ALIGN1);

				pid_init(&motor->pid);
				__HAL_TIM_SET_COUNTER(motor->commut_timer, 0);
				__HAL_TIM_SET_AUTORELOAD(motor->commut_timer, 1000);
				HAL_TIM_Base_Start(motor->commut_timer);
				HAL_TIMEx_ConfigCommutEvent_IT(motor->control_timer, motor->control_timer_itr, TIM_COMMUTATION_TRGI);
			}
        } break;
        case MOTOR_STATE_STARTUP_ALIGN1: {
			if(motor->vel_setpoint<VEL_THRESHOLD) {
				state_change(motor, MOTOR_STATE_PANIC);
			}

			if(time>=ALIGN_TIME) {
				motor->step = 1;
				motor->pulse = ALIGN_PULSE;

				state_change(motor, MOTOR_STATE_STARTUP_ALIGN2);
			}
        } break;
		case MOTOR_STATE_STARTUP_ALIGN2: {
			if(motor->vel_setpoint<VEL_THRESHOLD) {
				state_change(motor, MOTOR_STATE_PANIC);
			}

			if(time>=ALIGN_TIME) {
				motor->pulse = OPEN_LOOP_PULSE;
				__HAL_TIM_SET_AUTORELOAD(motor->commut_timer, OPEN_LOOP_RAMP_MAX);

				state_change(motor, MOTOR_STATE_STARTUP_OPEN_LOOP);

				motor->ramp_task = 0;
				motor->vel_task = 0;
				motor->zc_count = 0;
			}
        } break;
        case MOTOR_STATE_STARTUP_OPEN_LOOP: {
			if(software_timer(&motor->ramp_task, time, 1)) {
				const float t = time*0.001f;
				const float f = 1.f - expf(-OPEN_LOOP_RAMP_LAMBDA*t);
				const uint32_t T = CLAMP(OPEN_LOOP_RAMP_MIN/f, OPEN_LOOP_RAMP_MIN, OPEN_LOOP_RAMP_MAX);

				__HAL_TIM_SET_AUTORELOAD(motor->commut_timer, T);
				if(__HAL_TIM_GET_COUNTER(motor->commut_timer)>=T) {
					__HAL_TIM_SET_COUNTER(motor->commut_timer, T - 1);
				}
			}

			if(software_timer(&motor->vel_task, time, VEL_PERIOD)) {
				const uint32_t zc_count = motor->zc_count;
				motor->zc_count = 0;

				const float velocity = (2.f*PI*zc_count)/(6*MOTOR_POLE_PAIRS*VEL_PERIOD*0.001f);
				motor->vel = VEL_FILTER*motor->vel + (1.f - VEL_FILTER)*velocity;
			}

			if(motor->vel_setpoint<VEL_THRESHOLD) {
				state_change(motor, MOTOR_STATE_PANIC);
			}

			if(time>=OPEN_LOOP_RAMP_TIME) {
				if(motor->vel>VEL_THRESHOLD) {
					state_change(motor, MOTOR_STATE_RUNNING);
				} else {
					state_change(motor, MOTOR_STATE_PANIC);
				}
			}
        } break;
        case MOTOR_STATE_RUNNING: {
			if(software_timer(&motor->vel_task, time, VEL_PERIOD)) {
				const uint32_t zc_count = motor->zc_count;
				motor->zc_count = 0;

				const float velocity = (2.f*PI*zc_count)/(6*MOTOR_POLE_PAIRS*VEL_PERIOD*0.001f);
				motor->vel = VEL_FILTER*motor->vel + (1.f - VEL_FILTER)*velocity;
			}

			if(motor->vel_setpoint<VEL_THRESHOLD) {
				state_change(motor, MOTOR_STATE_PANIC);
			}

			if(motor->vel<VEL_THRESHOLD) {
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

void motor_set_vel(motor_t *motor, const float vel) {
    motor->vel_setpoint = vel;
}

void motor_commutation_callback(motor_t *motor, const TIM_HandleTypeDef *htim) {
	if(htim!=motor->control_timer) {
		return;
	}

	if(motor->state==MOTOR_STATE_RUNNING) {
		motor->pid.process = motor->vel;
		motor->pid.setpoint = motor->vel_setpoint;
		pid_calculate(&motor->pid);
		motor->pulse = CLAMP(
			CLOSED_LOOP_PULSE_MIN + motor->pid.value,
			CLOSED_LOOP_PULSE_MIN,
			CLOSED_LOOP_PULSE_MAX
		);
	}

	if((motor->state==MOTOR_STATE_STARTUP_OPEN_LOOP) || (motor->state==MOTOR_STATE_RUNNING)) {
		motor->step++;
		motor->step %=6;
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
}

void motor_interrupt_callback(motor_t *motor, const uint16_t pin) {
	const uint32_t counter = __HAL_TIM_GET_COUNTER(motor->commut_timer);
	const uint32_t autoreload = __HAL_TIM_GET_AUTORELOAD(motor->commut_timer);

	if(motor->state==MOTOR_STATE_RUNNING) {
		__HAL_TIM_SET_AUTORELOAD(motor->commut_timer, counter + autoreload/2);
	}

	motor->zc_count++;
}
