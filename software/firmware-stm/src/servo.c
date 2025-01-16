#include <stdint.h>
#include <stm32u5xx_hal.h>

#include "servo.h"

#define PI           3.141592653589f
#define DEG2RAD      (PI/180.f)
#define RAD2DEG      (180.f/PI)
#define MAX_POSITION (30.f*DEG2RAD)

void servo_init(servo_t *servo) {
    servo->position = 0;

    HAL_TIM_PWM_Start(servo->timer, servo->channel);
}

void servo_set_cmp(servo_t *servo, const uint32_t compare) {
    servo->position = NAN;

    __HAL_TIM_SET_COMPARE(servo->timer, servo->channel, compare);
}

void servo_set_pos(servo_t *servo, float angle) {
    if(angle>MAX_POSITION) {
        angle = MAX_POSITION;
    } else if(angle<-MAX_POSITION) {
        angle = -MAX_POSITION;
    }

    servo->position = angle;

    const uint32_t compare = 1500 + 500*(angle/(0.5f*PI));

    __HAL_TIM_SET_COMPARE(servo->timer, servo->channel, compare);
}
