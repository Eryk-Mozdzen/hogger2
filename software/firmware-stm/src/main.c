#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <jsmn/jsmn.h>

#include "stm32u5xx_hal.h"
#include "main.h"
#include "motor.h"
#include "protocol/protocol.h"

#define BUFFER_SIZE		(10*1024)
#define JSON_TOKEN_MAX  128

extern TIM_HandleTypeDef htim1;
extern TIM_HandleTypeDef htim2;
extern UART_HandleTypeDef huart1;
extern ADC_HandleTypeDef hadc1;

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

static jsmn_parser json_parser;
static jsmntok_t json_tokes[JSON_TOKEN_MAX];

typedef struct {
    float number;
} control_t;

static control_t control = {0};

void HAL_TIMEx_CommutCallback(TIM_HandleTypeDef *htim) {
	motor_commutation_callback(&motor, htim);
}

void HAL_GPIO_EXTI_Rising_Callback(uint16_t GPIO_Pin) {
	motor_interrupt_callback(&motor, GPIO_Pin);
}

void HAL_GPIO_EXTI_Falling_Callback(uint16_t GPIO_Pin) {
	motor_interrupt_callback(&motor, GPIO_Pin);
}

static void protocol_cb_transmit(const void *data, const uint32_t size) {
    HAL_UART_Transmit_DMA(&huart1, data, size);
}

static bool jsoneq(const char *json, const jsmntok_t *tok, const char *s) {
    return ((tok->type==JSMN_STRING) && ((int)strlen(s)==(tok->end-tok->start)) && (strncmp(json+tok->start, s, tok->end-tok->start)==0));
}

static void protocol_cb_receive(const uint8_t id, const void *payload, const uint32_t size) {
    if(id==0) {
        const char *json = payload;

        jsmn_init(&json_parser);
        memset(json_tokes, 0, sizeof(json_tokes));

        const int result = jsmn_parse(&json_parser, json, size, json_tokes, JSON_TOKEN_MAX);

        for(int i=0; i<result; i++) {
            if(jsoneq(json, &json_tokes[i], "number")) {
                control.number = atof(json + json_tokes[i + 1].start);
                i++;
            }
        }
    }

    if((id==1) && (size==1)) {
    	HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, *(uint8_t *)payload);
    }
}

void app_main() {

    motor_init(&motor);

    protocol.callback_tx = protocol_cb_transmit;
	protocol.callback_rx = protocol_cb_receive;
	protocol.fifo_rx.buffer = protocol_buffer_rx;
	protocol.fifo_rx.size = sizeof(protocol_buffer_rx);
	protocol.fifo_tx.buffer = protocol_buffer_tx;
	protocol.fifo_tx.size = sizeof(protocol_buffer_tx);
	protocol.decoded = protocol_buffer_decode;
	protocol.max = sizeof(protocol_buffer_decode);
	HAL_UART_Receive_DMA(&huart1, protocol.fifo_rx.buffer, protocol.fifo_rx.size);

    uint32_t task_state = 0;

    char json[1024];

    const char *robot_state[] = {
        "stopped",
        "manual",
        "autonomous",
    };

    const char *motor_state[] = {
        "idle",
        "startup",
        "startup",
        "startup",
        "running",
        "panic",
    };

    while(1) {
        const uint32_t time = HAL_GetTick();

        if((time - protocol.time)>=1) {
            protocol.time = time;
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

            HAL_ADC_Start(&hadc1);
            HAL_ADC_PollForConversion(&hadc1, 2);
            const uint32_t vbus_raw = HAL_ADC_GetValue(&hadc1);
            const float vbus_volt = (6.1f*3.3f*vbus_raw)/((float)(1<<14));

            sprintf(json,
                "{\n"
                "    \"timestamp\": %lu,\n"
                "    \"state\": \"%s\",\n"
                "    \"battery\": %.2f,\n"
                "    \"hog1\": {\n"
                "        \"servo x\": %.3f,\n"
                "        \"servo y\": %.3f,\n"
                "        \"motor\": %.3f,\n"
                "        \"state\": \"%s\"\n"
                "    },\n"
                "    \"hog2\": {\n"
                "        \"servo x\": %.3f,\n"
                "        \"servo y\": %.3f,\n"
                "        \"motor\": %.3f,\n"
                "        \"state\": \"%s\"\n"
                "    }\n"
                "}",
                HAL_GetTick(),
                robot_state[0],
                vbus_volt,
                0.f,
                0.f,
                motor.vel,
                motor_state[motor.state],
                0.f,
                0.f,
                0.f,
                motor_state[0]
            );

            protocol_enqueue(&protocol, 0, json, strlen(json)+1);
        }

        if(!HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_2)) {
            motor_set_vel(&motor, 200);
        }

        motor_tick(&motor);
    }
}
