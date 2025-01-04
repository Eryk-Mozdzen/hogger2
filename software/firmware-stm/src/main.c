#include <stdio.h>
#include <string.h>

#include "stm32u5xx_hal.h"
#include "main.h"
#include "motor.h"
#include "protocol/protocol.h"

#define BUFFER_SIZE		(10*1024)

extern TIM_HandleTypeDef htim1;
extern TIM_HandleTypeDef htim2;
extern UART_HandleTypeDef huart1;

static motor_t motor = {
	.control_timer = &htim1,
	.control_timer_itr = TIM_TS_ITR1,
	.commut_timer = &htim2,
    .bemf[MOTOR_PHASE_U] = MOTOR1_BEMF_U_Pin,
    .bemf[MOTOR_PHASE_V] = MOTOR1_BEMF_V_Pin,
    .bemf[MOTOR_PHASE_W] = MOTOR1_BEMF_W_Pin,
};

static protocol_t protocol = PROTOCOL_INIT;
static uint8_t protocol_buffer_rx[BUFFER_SIZE];
static uint8_t protocol_buffer_tx[BUFFER_SIZE];
static uint8_t protocol_buffer_decode[BUFFER_SIZE];

void HAL_TIMEx_CommutCallback(TIM_HandleTypeDef *htim) {
	motor_commutation_callback(&motor, htim);
}

void HAL_GPIO_EXTI_Rising_Callback(uint16_t GPIO_Pin) {
	motor_interrupt_callback(&motor, GPIO_Pin);
}

void HAL_GPIO_EXTI_Falling_Callback(uint16_t GPIO_Pin) {
	motor_interrupt_callback(&motor, GPIO_Pin);
}

static void protocol_cb_transmit(void *user, const void *data, const uint32_t size) {
    (void)user;
    HAL_UART_Transmit_DMA(&huart1, data, size);
}

static void protocol_cb_receive(void *user, const uint8_t id, const uint32_t time, const void *payload, const uint32_t size) {
    (void)user;
    (void)time;
    (void)payload;

    if((id==1) && (size==1)) {
    	HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, *(uint8_t *)payload);
    }
}

static uint32_t protocol_cb_time(void *user) {
	(void)user;
	return HAL_GetTick();
}

void app_main() {

    motor_init(&motor);

    protocol.callback_tx = protocol_cb_transmit;
	protocol.callback_rx = protocol_cb_receive;
	protocol.callback_time = protocol_cb_time;
	protocol.fifo_rx.buffer = protocol_buffer_rx;
	protocol.fifo_rx.size = sizeof(protocol_buffer_rx);
	protocol.fifo_tx.buffer = protocol_buffer_tx;
	protocol.fifo_tx.size = sizeof(protocol_buffer_tx);
	protocol.decoded = protocol_buffer_decode;
	protocol.max = sizeof(protocol_buffer_decode);

	HAL_UART_Receive_DMA(&huart1, protocol.fifo_rx.buffer, protocol.fifo_rx.size);

    uint32_t task_protocol = 0;
    uint32_t task_state = 0;

    while(1) {
        const uint32_t time = HAL_GetTick();

        if((time - task_protocol)>=1) {
            task_protocol = time;
            protocol.fifo_rx.write = (protocol.fifo_rx.size - __HAL_DMA_GET_COUNTER(huart1.hdmarx)) % protocol.fifo_rx.size;
            protocol.available = (HAL_DMA_GetState(huart1.hdmatx)==HAL_DMA_STATE_READY);
            protocol_process(&protocol);
        }

        if((time - protocol.time_last_rx)>=1000) {
            protocol.time_last_rx = time;
            HAL_UART_Receive_DMA(&huart1, protocol.fifo_rx.buffer, protocol.fifo_rx.size);
        }

        if((time - task_state)>=10) {
            task_state = time;
            char json[1024];
            sprintf(json, "{\"STM timestamp\": %lu}", HAL_GetTick());

            protocol_enqueue(&protocol, 0, json, strlen(json)+1);
        }

        if(!HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_2)) {
            motor_set_vel(&motor, 200);
        }

        motor_tick(&motor);
    }
}
