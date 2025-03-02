#include <main.h>
#include <stm32u5xx_hal.h>

#include "actuate/BLDC.hpp"
#include "freertos/Task.hpp"

extern TIM_HandleTypeDef htim1;
extern TIM_HandleTypeDef htim2;
extern TIM_HandleTypeDef htim3;
extern TIM_HandleTypeDef htim8;

class Motors : freertos::TaskClass<1024> {
    static BLDC::Config getConfig(const int number);

    void task();

public:
    BLDC motor1;
    BLDC motor2;

    Motors();
};

Motors motors;

static void commutationISR(TIM_HandleTypeDef *htim) {
    motors.motor1.commutationISR(htim);
    motors.motor2.commutationISR(htim);
}

static void periodISR(TIM_HandleTypeDef *htim) {
    motors.motor1.sampleISR(htim);
    motors.motor2.sampleISR(htim);
}

Motors::Motors()
    : TaskClass{"motors", 15}, motor1{"motor_1", getConfig(1)}, motor2{"motor_2", getConfig(2)} {
}

BLDC::Config Motors::getConfig(const int number) {
    BLDC::Config config;

    switch(number) {
        case 1: {
            config.control_timer = &htim1;
            config.control_timer_itr = TIM_TS_ITR1;
            config.commut_timer = &htim2;
            config.bemf[BLDC::Phase::U] = {MOTOR1_BEMF_U_Pin, MOTOR1_BEMF_U_GPIO_Port};
            config.bemf[BLDC::Phase::V] = {MOTOR1_BEMF_V_Pin, MOTOR1_BEMF_V_GPIO_Port};
            config.bemf[BLDC::Phase::W] = {MOTOR1_BEMF_W_Pin, MOTOR1_BEMF_W_GPIO_Port};
        } break;
        case 2: {
            config.control_timer = &htim8;
            config.control_timer_itr = TIM_TS_ITR2;
            config.commut_timer = &htim3;
            config.bemf[BLDC::Phase::U] = {MOTOR2_BEMF_U_Pin, MOTOR2_BEMF_U_GPIO_Port};
            config.bemf[BLDC::Phase::V] = {MOTOR2_BEMF_V_Pin, MOTOR2_BEMF_V_GPIO_Port};
            config.bemf[BLDC::Phase::W] = {MOTOR2_BEMF_W_Pin, MOTOR2_BEMF_W_GPIO_Port};
        } break;
    }

    return config;
}

void Motors::task() {
    uint32_t time = 0;

    HAL_TIM_RegisterCallback(&htim1, HAL_TIM_COMMUTATION_CB_ID, commutationISR);
    HAL_TIM_RegisterCallback(&htim8, HAL_TIM_COMMUTATION_CB_ID, commutationISR);
    HAL_TIM_RegisterCallback(&htim1, HAL_TIM_PERIOD_ELAPSED_CB_ID, periodISR);
    HAL_TIM_RegisterCallback(&htim8, HAL_TIM_PERIOD_ELAPSED_CB_ID, periodISR);

    while(true) {
        motor1.tick();
        motor2.tick();

        // TODO: transmit velocity to measurement queue

        vTaskDelayUntil(&time, 1);
    }
}

namespace motor {

void setVelocity(const float vel1, const float vel2) {
    motors.motor1.setVelocity(vel1);
    motors.motor2.setVelocity(vel2);
}

}
