#include <stdint.h>
#include <stdbool.h>

#include "stm32u5xx_hal.h"
#include "motor.h"

#define MOTOR_POLE_PAIRS		7
#define PI						3.141592653589f

#define IDLE_THRESHOLD			200.f	// [rad/s]

#define ALIGN_TIME              100     // [ms]
#define ALIGN_PULSE             0.3f    // []

#define OPEN_LOOP_PULSE         0.2f    // []
#define OPEN_LOOP_VEL_THRESHOLD 300.f   // [rad/s]
#define OPEN_LOOP_TIME          1000    // [ms]
#define OPEN_LOOP_COMMUT_PERIOD 4		// [ms]
#define OPEN_LOOP_ZC_PERIOD		100	    // [ms]

#define CLOSED_LOOP_VEL_MIN     200.f   // [rad/s]
#define CLOSED_LOOP_KP          1.f
#define CLOSED_LOOP_KI          0.f
#define CLOSED_LOOP_KD          0.f

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
	HAL_NVIC_DisableIRQ(motor->bemf[MOTOR_PHASE_U].irq);
	HAL_NVIC_DisableIRQ(motor->bemf[MOTOR_PHASE_V].irq);
	HAL_NVIC_DisableIRQ(motor->bemf[MOTOR_PHASE_W].irq);

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
	motor->tick_last_time = 0;
	motor->zc_last_time = 0;
}

static bool software_timer(uint32_t *prev, const uint32_t time, const uint32_t period) {
	if((time - *prev)>=period) {
		*prev = time;
		return true;
	}

	return false;
}

static float pid_calculate(motor_pid_t *pid, const float setpoint, const float process) {
	const float error = process - setpoint;

	return CLOSED_LOOP_KP*error;// + CLOSED_LOOP_KI*pid->error_integral + CLOSED_LOOP_KD*error_derivative;
}

void motor_init(motor_t *motor) {
    motor->state = MOTOR_STATE_IDLE;
    motor->step = 0;
	motor->pulse = 0;
	motor->vel = 0;
	motor->vel_setpoint = 0;
	motor->zc_count = 0;

	shutdown(motor);

	HAL_TIMEx_ConfigCommutEvent_IT(motor->control_timer, TIM_TS_NONE, TIM_COMMUTATION_SOFTWARE);
	HAL_TIM_Base_Start_IT(motor->timebase_timer);
}

void motor_tick(motor_t *motor) {
	const uint32_t time = HAL_GetTick() - motor->state_start_time;

    switch(motor->state) {
        case MOTOR_STATE_IDLE: {
			if(motor->vel_setpoint>IDLE_THRESHOLD) {
				state_change(motor, MOTOR_STATE_STARTUP_ALIGN1);

				motor->step = 0;
				motor->pulse = ALIGN_PULSE;
				HAL_TIM_GenerateEvent(motor->control_timer, TIM_EVENTSOURCE_COM);
			}
        } break;
        case MOTOR_STATE_STARTUP_ALIGN1: {
			if(time>=ALIGN_TIME) {
				state_change(motor, MOTOR_STATE_STARTUP_ALIGN2);

				motor->step = 1;
				motor->pulse = ALIGN_PULSE;
				HAL_TIM_GenerateEvent(motor->control_timer, TIM_EVENTSOURCE_COM);
			}
        } break;
		case MOTOR_STATE_STARTUP_ALIGN2: {
			if(time>=ALIGN_TIME) {
				state_change(motor, MOTOR_STATE_STARTUP_OPEN_LOOP);

				motor->zc_count = 0;
			}
        } break;
        case MOTOR_STATE_STARTUP_OPEN_LOOP: {
			if(software_timer(&motor->tick_last_time, time, OPEN_LOOP_COMMUT_PERIOD)) {
				motor->step++;
				motor->step %=6;
				motor->pulse = OPEN_LOOP_PULSE;
				HAL_TIM_GenerateEvent(motor->control_timer, TIM_EVENTSOURCE_COM);
			}

			if(software_timer(&motor->zc_last_time, time, OPEN_LOOP_ZC_PERIOD)) {
				motor->vel = 2.f*PI*motor->zc_count/(6*MOTOR_POLE_PAIRS*OPEN_LOOP_ZC_PERIOD*0.001f);
				motor->zc_count = 0;
			}

			if(time>=OPEN_LOOP_TIME) {
				state_change(motor, MOTOR_STATE_PANIC);
			}

			if(motor->vel>=OPEN_LOOP_VEL_THRESHOLD) {
				state_change(motor, MOTOR_STATE_RUNNING);
			}
        } break;
        case MOTOR_STATE_RUNNING: {
			if(motor->vel<CLOSED_LOOP_VEL_MIN) {
				state_change(motor, MOTOR_STATE_PANIC);
			}
        } break;
		case MOTOR_STATE_PANIC: {
			motor->pulse = 0;
			motor->vel_setpoint = 0;
			shutdown(motor);

			state_change(motor, MOTOR_STATE_IDLE);
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

	switch(motor->step) {
		case 0: {
			HAL_NVIC_DisableIRQ(motor->bemf[MOTOR_PHASE_U].irq);
			HAL_NVIC_DisableIRQ(motor->bemf[MOTOR_PHASE_V].irq);
			HAL_NVIC_DisableIRQ(motor->bemf[MOTOR_PHASE_W].irq);

			config_pwm(motor, TIM_CHANNEL_1);
			config_oc(motor, TIM_CHANNEL_2, TIM_OCMODE_FORCED_ACTIVE);
			config_oc(motor, TIM_CHANNEL_3, TIM_OCMODE_FORCED_INACTIVE);

			HAL_NVIC_EnableIRQ(motor->bemf[MOTOR_PHASE_W].irq);
		} break;
		case 1: {
			HAL_NVIC_DisableIRQ(motor->bemf[MOTOR_PHASE_U].irq);
			HAL_NVIC_DisableIRQ(motor->bemf[MOTOR_PHASE_V].irq);
			HAL_NVIC_DisableIRQ(motor->bemf[MOTOR_PHASE_W].irq);

			config_pwm(motor, TIM_CHANNEL_1);
			config_oc(motor, TIM_CHANNEL_2, TIM_OCMODE_FORCED_INACTIVE);
			config_oc(motor, TIM_CHANNEL_3, TIM_OCMODE_FORCED_ACTIVE);

			HAL_NVIC_EnableIRQ(motor->bemf[MOTOR_PHASE_V].irq);
		} break;
		case 2: {
			HAL_NVIC_DisableIRQ(motor->bemf[MOTOR_PHASE_U].irq);
			HAL_NVIC_DisableIRQ(motor->bemf[MOTOR_PHASE_V].irq);
			HAL_NVIC_DisableIRQ(motor->bemf[MOTOR_PHASE_W].irq);

			config_oc(motor, TIM_CHANNEL_1, TIM_OCMODE_FORCED_INACTIVE);
			config_pwm(motor, TIM_CHANNEL_2);
			config_oc(motor, TIM_CHANNEL_3, TIM_OCMODE_FORCED_ACTIVE);

			HAL_NVIC_EnableIRQ(motor->bemf[MOTOR_PHASE_U].irq);
		} break;
		case 3: {
			HAL_NVIC_DisableIRQ(motor->bemf[MOTOR_PHASE_U].irq);
			HAL_NVIC_DisableIRQ(motor->bemf[MOTOR_PHASE_V].irq);
			HAL_NVIC_DisableIRQ(motor->bemf[MOTOR_PHASE_W].irq);

			config_oc(motor, TIM_CHANNEL_1, TIM_OCMODE_FORCED_ACTIVE);
			config_pwm(motor, TIM_CHANNEL_2);
			config_oc(motor, TIM_CHANNEL_3, TIM_OCMODE_FORCED_INACTIVE);

			HAL_NVIC_EnableIRQ(motor->bemf[MOTOR_PHASE_W].irq);
		} break;
		case 4: {
			HAL_NVIC_DisableIRQ(motor->bemf[MOTOR_PHASE_U].irq);
			HAL_NVIC_DisableIRQ(motor->bemf[MOTOR_PHASE_V].irq);
			HAL_NVIC_DisableIRQ(motor->bemf[MOTOR_PHASE_W].irq);

			config_oc(motor, TIM_CHANNEL_1, TIM_OCMODE_FORCED_ACTIVE);
			config_oc(motor, TIM_CHANNEL_2, TIM_OCMODE_FORCED_INACTIVE);
			config_pwm(motor, TIM_CHANNEL_3);

			HAL_NVIC_EnableIRQ(motor->bemf[MOTOR_PHASE_V].irq);
		} break;
		case 5: {
			HAL_NVIC_DisableIRQ(motor->bemf[MOTOR_PHASE_U].irq);
			HAL_NVIC_DisableIRQ(motor->bemf[MOTOR_PHASE_V].irq);
			HAL_NVIC_DisableIRQ(motor->bemf[MOTOR_PHASE_W].irq);

			config_oc(motor, TIM_CHANNEL_1, TIM_OCMODE_FORCED_INACTIVE);
			config_oc(motor, TIM_CHANNEL_2, TIM_OCMODE_FORCED_ACTIVE);
			config_pwm(motor, TIM_CHANNEL_3);

			HAL_NVIC_EnableIRQ(motor->bemf[MOTOR_PHASE_U].irq);
		} break;
	}

	__HAL_TIM_SET_COUNTER(motor->timebase_timer, 0);
	__HAL_TIM_SET_AUTORELOAD(motor->timebase_timer, 65535);
}

void motor_autoreload_callback(motor_t *motor, const TIM_HandleTypeDef *htim) {
	if(htim!=motor->timebase_timer) {
		return;
	}

	if(motor->state==MOTOR_STATE_RUNNING) {
		motor->step++;
		motor->step %=6;
		motor->pulse = OPEN_LOOP_PULSE;
		HAL_TIM_GenerateEvent(motor->control_timer, TIM_EVENTSOURCE_COM);
	}
}

void motor_interrupt_callback(motor_t *motor, const uint16_t pin) {
	if((pin!=motor->bemf[MOTOR_PHASE_U].pin) && (pin!=motor->bemf[MOTOR_PHASE_V].pin) && (pin!=motor->bemf[MOTOR_PHASE_W].pin)) {
		return;
	}

	motor->zc_count++;

	if(motor->state==MOTOR_STATE_RUNNING) {
		const uint32_t counter = __HAL_TIM_GET_COUNTER(motor->timebase_timer);

		const float process = counter*0.0001f;
		const float setpoint = PI/(motor->vel_setpoint*6*MOTOR_POLE_PAIRS);
		const float value = pid_calculate(&motor->pid, setpoint, process);

		__HAL_TIM_SET_COUNTER(motor->timebase_timer, 0);
		__HAL_TIM_SET_AUTORELOAD(motor->timebase_timer, counter + value*10000);
	}
}
