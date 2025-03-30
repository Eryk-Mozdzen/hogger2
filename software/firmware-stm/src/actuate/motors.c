#include <math.h>
#include <stm32h5xx_hal.h>
#include <string.h>

#include "actuate/motor.h"
#include "com/telemetry.h"
#include "utils/task.h"

#define VEL_MIN -1000.f
#define VEL_MAX 1000.f

extern ADC_HandleTypeDef hadc1;
extern ADC_HandleTypeDef hadc2;
extern TIM_HandleTypeDef htim1;
extern TIM_HandleTypeDef htim2;
extern TIM_HandleTypeDef htim5;
extern TIM_HandleTypeDef htim8;

static motor_t motor1 = {
    .control_timer = &htim8,
    .control_timer_itr = TIM_TS_ITR1,
    .commut_timer = &htim2,
    .bemf_adc = &hadc1,
};

static motor_t motor2 = {
    .control_timer = &htim1,
    .control_timer_itr = TIM_TS_ITR4,
    .commut_timer = &htim5,
    .bemf_adc = &hadc2,
};

static void isr_commutation(TIM_HandleTypeDef *htim) {
    motor_commutation_callback(&motor1, htim);
    motor_commutation_callback(&motor2, htim);
}

static void isr_conversion(ADC_HandleTypeDef *hadc) {
    motor_sample_callback(&motor1, hadc);
    motor_sample_callback(&motor2, hadc);
}

static float validate(const float velocity) {
    if(isnan(velocity)) {
        return 0;
    }

    if(isinf(velocity)) {
        return 0;
    }

    if(velocity > VEL_MAX) {
        return VEL_MAX;
    }

    if(velocity < VEL_MIN) {
        return VEL_MIN;
    }

    return velocity;
}

void motors_set_velocity(const float psi1_dot, const float psi2_dot) {
    motor1.vel_setpoint = validate(psi1_dot);
    motor2.vel_setpoint = validate(psi2_dot);
}

static void init() {
    HAL_TIM_RegisterCallback(&htim1, HAL_TIM_COMMUTATION_CB_ID, isr_commutation);
    HAL_TIM_RegisterCallback(&htim8, HAL_TIM_COMMUTATION_CB_ID, isr_commutation);
    HAL_ADC_RegisterCallback(&hadc1, HAL_ADC_INJ_CONVERSION_COMPLETE_CB_ID, isr_conversion);
    HAL_ADC_RegisterCallback(&hadc2, HAL_ADC_INJ_CONVERSION_COMPLETE_CB_ID, isr_conversion);

    motor_init(&motor1);
    motor_init(&motor2);
}

static void loop() {
    motor_tick(&motor1);
    motor_tick(&motor2);
}

static void serialize(cmp_ctx_t *cmp, void *context) {
    const motor_t *motor = context;

    const char *stateStr[] = {
        "idle", "startup_align_1", "startup_align_2", "startup_ramp", "running", "panic",
    };

    cmp_write_map(cmp, 4);
    cmp_write_str(cmp, "state", 5);
    cmp_write_str(cmp, stateStr[motor->state], strlen(stateStr[motor->state]));
    cmp_write_str(cmp, "vel_ref", 7);
    cmp_write_float(cmp, motor->vel_setpoint);
    cmp_write_str(cmp, "vel", 3);
    cmp_write_float(cmp, motor->vel);
    cmp_write_str(cmp, "load", 4);
    cmp_write_float(cmp, motor->pulse);
}

TASK_REGISTER_INIT(init)
TASK_REGISTER_PERIODIC(loop, 100)
TELEMETRY_REGISTER("motor_1", serialize, &motor1)
TELEMETRY_REGISTER("motor_2", serialize, &motor2)
