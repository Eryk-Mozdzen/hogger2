#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stm32u5xx_hal.h>
#include <cmp/cmp.h>

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

typedef enum {
    REFERENCE_TYPE_NONE,
    REFERENCE_TYPE_CONFIGURATION,
    REFERENCE_TYPE_TRAJECTORY,
} reference_type_t;

typedef struct {
    reference_type_t type;
    union {
        float configuration[6];
        float trajectory[3];
    };
} reference_t;

static reference_t reference = {.type = REFERENCE_TYPE_NONE};

void HAL_TIMEx_CommutCallback(TIM_HandleTypeDef *htim) {
	motor_commutation_callback(&motor, htim);
}

void HAL_GPIO_EXTI_Rising_Callback(uint16_t GPIO_Pin) {
	motor_interrupt_callback(&motor, GPIO_Pin);
}

void HAL_GPIO_EXTI_Falling_Callback(uint16_t GPIO_Pin) {
	motor_interrupt_callback(&motor, GPIO_Pin);
}

typedef struct {
    uint8_t *buffer;
    size_t capacity;
    size_t position;
    size_t size;
} buffer_t;

size_t buffer_writer(cmp_ctx_t *ctx, const void *data, size_t count) {
    buffer_t *buf = (buffer_t *)ctx->buf;

    if((buf->position+count)>buf->capacity) {
        return 0;
    }

    memcpy(buf->buffer+buf->position, data, count);
    buf->position +=count;
    buf->size = buf->position;
    return count;
}

bool buffer_reader(cmp_ctx_t *ctx, void *data, size_t count) {
    buffer_t *buf = (buffer_t *)ctx->buf;

    if((buf->position+count)>buf->size) {
        return false;
    }

    memcpy(data, buf->buffer + buf->position, count);
    buf->position +=count;
    return true;
}

static void protocol_cb_transmit(const void *data, const uint32_t size) {
    HAL_UART_Transmit_DMA(&huart1, data, size);
}

static void protocol_cb_receive(const uint8_t id, const void *payload, const uint32_t size) {
    if(id==0) {
        buffer_t buffer = {
            .buffer = (void *)payload,
            .capacity = size,
            .size = size,
            .position = 0,
        };
        cmp_ctx_t cmp;
        cmp_init(&cmp, &buffer, buffer_reader, NULL, NULL);

        uint32_t map_size = 0;
        if(!cmp_read_map(&cmp, &map_size)) {
            return;
        }
        if(map_size!=2) {
            return;
        }

        char key[32] = {0};
        uint32_t key_size = sizeof(key);
        if(!cmp_read_str(&cmp, key, &key_size)) {
            return;
        }
        if(strncmp(key, "command", key_size)!=0) {
            return;
        }

        char command[32] = {0};
        uint32_t command_size = sizeof(command);
        if(!cmp_read_str(&cmp, command, &command_size)) {
            return;
        }
        if(strncmp(command, "manual", command_size)!=0) {
            return;
        }

        key_size = sizeof(key);
        if(!cmp_read_str(&cmp, key, &key_size)) {
            return;
        }
        if(strncmp(key, "ref_cfg", key_size)!=0) {
            return;
        }

        uint32_t array_size = 0;
        if(!cmp_read_array(&cmp, &array_size)) {
            return;
        }
        if(array_size!=6) {
            return;
        }

        for(uint8_t i=0; i<6; i++) {
            float floating = 0;
            if(!cmp_read_float(&cmp, &floating)) {
                reference.configuration[i] = floating;
                return;
            }
        }

        reference.type = REFERENCE_TYPE_CONFIGURATION;
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

    uint8_t msgpack[1024];

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

            buffer_t buffer = {
                .buffer = msgpack,
                .capacity = sizeof(msgpack),
                .size = 0,
                .position = 0,
            };
            cmp_ctx_t cmp = {0};
            cmp_init(&cmp, &buffer, NULL, NULL, buffer_writer);

            cmp_write_map(&cmp, 5);
            cmp_write_str(&cmp, "time", 4);
            cmp_write_u32(&cmp, time);
            cmp_write_str(&cmp, "state", 5);
            cmp_write_str(&cmp, robot_state[0], strlen(robot_state[0]));
            cmp_write_str(&cmp, "supply", 6);
            cmp_write_float(&cmp, vbus_volt);

            cmp_write_str(&cmp, "hog1", 4);
            cmp_write_map(&cmp, 4);
            cmp_write_str(&cmp, "servox", 6);
            cmp_write_float(&cmp, 0);
            cmp_write_str(&cmp, "servoy", 6);
            cmp_write_float(&cmp, 0);
            cmp_write_str(&cmp, "motor", 5);
            cmp_write_float(&cmp, motor.vel);
            cmp_write_str(&cmp, "state", 5);
            cmp_write_str(&cmp, motor_state[motor.state], strlen(motor_state[motor.state]));

            cmp_write_str(&cmp, "hog2", 4);
            cmp_write_map(&cmp, 4);
            cmp_write_str(&cmp, "servox", 6);
            cmp_write_float(&cmp, 0);
            cmp_write_str(&cmp, "servoy", 6);
            cmp_write_float(&cmp, 0);
            cmp_write_str(&cmp, "motor", 5);
            cmp_write_float(&cmp, 0);
            cmp_write_str(&cmp, "state", 5);
            cmp_write_str(&cmp, motor_state[0], strlen(motor_state[0]));

            protocol_enqueue(&protocol, 0, buffer.buffer, buffer.size);
        }

        if(!HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_2)) {
            motor_set_vel(&motor, 200);
        }

        motor_tick(&motor);
    }
}
