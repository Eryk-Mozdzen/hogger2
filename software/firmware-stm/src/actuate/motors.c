#include <main.h>
#include <string.h>

#include "actuate/motor.h"
#include "com/telemetry.h"
#include "utils/task.h"

extern TIM_HandleTypeDef htim1;
extern TIM_HandleTypeDef htim2;
extern TIM_HandleTypeDef htim3;
extern TIM_HandleTypeDef htim8;

static motor_t motor1 = {
    .control_timer = &htim1,
    .control_timer_itr = TIM_TS_ITR1,
    .commut_timer = &htim2,
    .bemf[MOTOR_PHASE_U].pin = MOTOR1_BEMF_U_Pin,
    .bemf[MOTOR_PHASE_U].port = MOTOR1_BEMF_U_GPIO_Port,
    .bemf[MOTOR_PHASE_V].pin = MOTOR1_BEMF_V_Pin,
    .bemf[MOTOR_PHASE_V].port = MOTOR1_BEMF_V_GPIO_Port,
    .bemf[MOTOR_PHASE_W].pin = MOTOR1_BEMF_W_Pin,
    .bemf[MOTOR_PHASE_W].port = MOTOR1_BEMF_W_GPIO_Port,
};

static motor_t motor2 = {
    .control_timer = &htim8,
    .control_timer_itr = TIM_TS_ITR2,
    .commut_timer = &htim3,
    .bemf[MOTOR_PHASE_U].pin = MOTOR2_BEMF_U_Pin,
    .bemf[MOTOR_PHASE_U].port = MOTOR2_BEMF_U_GPIO_Port,
    .bemf[MOTOR_PHASE_V].pin = MOTOR2_BEMF_V_Pin,
    .bemf[MOTOR_PHASE_V].port = MOTOR2_BEMF_V_GPIO_Port,
    .bemf[MOTOR_PHASE_W].pin = MOTOR2_BEMF_W_Pin,
    .bemf[MOTOR_PHASE_W].port = MOTOR2_BEMF_W_GPIO_Port,
};

static void isr_commutation(TIM_HandleTypeDef *htim) {
    motor_commutation_callback(&motor1, htim);
    motor_commutation_callback(&motor2, htim);
}

static void isr_period_elapsed(TIM_HandleTypeDef *htim) {
    motor_sample_callback(&motor1, htim);
    motor_sample_callback(&motor2, htim);
}

void motors_set_velocity(const float vel1, const float vel2) {
    motor1.vel_setpoint = vel1;
    motor2.vel_setpoint = vel2;
}

static void init() {
    HAL_TIM_RegisterCallback(&htim1, HAL_TIM_COMMUTATION_CB_ID, isr_commutation);
    HAL_TIM_RegisterCallback(&htim8, HAL_TIM_COMMUTATION_CB_ID, isr_commutation);
    HAL_TIM_RegisterCallback(&htim1, HAL_TIM_PERIOD_ELAPSED_CB_ID, isr_period_elapsed);
    HAL_TIM_RegisterCallback(&htim8, HAL_TIM_PERIOD_ELAPSED_CB_ID, isr_period_elapsed);

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
