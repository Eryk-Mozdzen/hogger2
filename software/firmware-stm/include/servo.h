#ifndef SERVO_H
#define SERVO_H

#include <stdint.h>
#include <stm32u5xx_hal.h>

typedef struct {
    TIM_HandleTypeDef *timer;
    uint32_t channel;
    float position;
} servo_t;

void servo_init(servo_t *servo);
void servo_set_cmp(servo_t *servo, const uint32_t compare);
void servo_set_pos(servo_t *servo, float angle);

#endif
