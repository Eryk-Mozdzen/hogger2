#include <stdint.h>

#include "stm32u5xx_hal.h"
#include "motor.h"

#define MOTOR_POLE_PAIRS		7

#define IDLE_THRESHOLD			100.f	// [rad/s]

#define ALIGN_TIME              100     // [ms]
#define ALIGN_PULSE             0.3f    // []

#define OPEN_LOOP_PULSE         0.2f    // []
#define OPEN_LOOP_VEL_THRESHOLD 1000.f  // [rad/s]
#define OPEN_LOOP_TIME          1000    // [ms]
#define OPEN_LOOP_COMMUT_PERIOD 4		// [ms]

#define CLOSED_LOOP_VEL_MIN     500.f   // [rad/s]
#define CLOSED_LOOP_KP          1.f
#define CLOSED_LOOP_KI          0.f

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
}

static uint32_t state_time(const motor_t *motor) {
	const uint32_t time = HAL_GetTick();

	return (time - motor->state_start_time);
}

void motor_init(motor_t *motor) {
    motor->state = MOTOR_STATE_IDLE;
    motor->step = 0;
	motor->pulse = 0;
	motor->vel = 0;
	motor->vel_setpoint = 0;

	shutdown(motor);

	HAL_TIMEx_ConfigCommutEvent_IT(motor->control_timer, TIM_TS_NONE, TIM_COMMUTATION_SOFTWARE);
	HAL_TIM_Base_Start(motor->timebase_timer);
	HAL_NVIC_EnableIRQ(motor->bemf[MOTOR_PHASE_W].irq);
}

void motor_tick(motor_t *motor) {
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
			if(state_time(motor)>=ALIGN_TIME) {
				state_change(motor, MOTOR_STATE_STARTUP_ALIGN2);

				motor->step = 1;
				motor->pulse = ALIGN_PULSE;
				HAL_TIM_GenerateEvent(motor->control_timer, TIM_EVENTSOURCE_COM);
			}
        } break;
		case MOTOR_STATE_STARTUP_ALIGN2: {
			if(state_time(motor)>=ALIGN_TIME) {
				state_change(motor, MOTOR_STATE_STARTUP_OPEN_LOOP);
			}
        } break;
        case MOTOR_STATE_STARTUP_OPEN_LOOP: {

			const uint32_t time = state_time(motor);
			if((time - motor->tick_last_time)>=OPEN_LOOP_COMMUT_PERIOD) {
				motor->tick_last_time = time;

				motor->step++;
				motor->step %=6;
				motor->pulse = OPEN_LOOP_PULSE;
				HAL_TIM_GenerateEvent(motor->control_timer, TIM_EVENTSOURCE_COM);
			}

			if(state_time(motor)>=OPEN_LOOP_TIME) {
				state_change(motor, MOTOR_STATE_PANIC);
			}

			//if(feedback is stable) {
			//	state_change(motor, MOTOR_STATE_RUNNING);
			//}
        } break;
        case MOTOR_STATE_RUNNING: {

        } break;
		case MOTOR_STATE_PANIC: {
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
	if(pin==motor->bemf[MOTOR_PHASE_U].pin) {
		HAL_NVIC_DisableIRQ(motor->bemf[MOTOR_PHASE_U].irq);
		HAL_NVIC_EnableIRQ(motor->bemf[MOTOR_PHASE_W].irq);

		return;
	}

	if(pin==motor->bemf[MOTOR_PHASE_V].pin) {
		HAL_NVIC_DisableIRQ(motor->bemf[MOTOR_PHASE_V].irq);
		HAL_NVIC_EnableIRQ(motor->bemf[MOTOR_PHASE_U].irq);

		return;
	}

	if(pin==motor->bemf[MOTOR_PHASE_W].pin) {
		HAL_NVIC_DisableIRQ(motor->bemf[MOTOR_PHASE_W].irq);
		HAL_NVIC_EnableIRQ(motor->bemf[MOTOR_PHASE_V].irq);

		return;
	}
}
