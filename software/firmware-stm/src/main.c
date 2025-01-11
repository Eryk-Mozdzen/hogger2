#include <stm32u5xx_hal.h>
#include <lrcp/frame.h>

#include "main.h"
#include "motor.h"
#include "mpu6050_regs.h"
#include "pmw3901_regs.h"
#include "serial.h"
#include "serialize.h"
#include "robot.h"
#include "reference.h"
#include "imu.h"
#include "flow.h"

extern TIM_HandleTypeDef htim1;
extern TIM_HandleTypeDef htim2;
extern TIM_HandleTypeDef htim3;
extern TIM_HandleTypeDef htim8;
extern UART_HandleTypeDef huart1;
extern ADC_HandleTypeDef hadc1;
extern I2C_HandleTypeDef hi2c1;
extern SPI_HandleTypeDef hspi2;

static motor_t motor1 = {
	.control_timer = &htim1,
	.control_timer_itr = TIM_TS_ITR1,
	.commut_timer = &htim2,
    .bemf[MOTOR_PHASE_U] = MOTOR1_BEMF_U_Pin,
    .bemf[MOTOR_PHASE_V] = MOTOR1_BEMF_V_Pin,
    .bemf[MOTOR_PHASE_W] = MOTOR1_BEMF_W_Pin,
};

static motor_t motor2 = {
	.control_timer = &htim8,
	.control_timer_itr = TIM_TS_ITR2,
	.commut_timer = &htim3,
    .bemf[MOTOR_PHASE_U] = MOTOR2_BEMF_U_Pin,
    .bemf[MOTOR_PHASE_V] = MOTOR2_BEMF_V_Pin,
    .bemf[MOTOR_PHASE_W] = MOTOR2_BEMF_W_Pin,
};

static imu_t imu = {
    .i2c = &hi2c1,
    .interrupt = IMU_INT_Pin,
};

static flow_t flow = {
    .spi = &hspi2,
    .chip_select.port = FLOW_CS_GPIO_Port,
    .chip_select.pin = FLOW_CS_Pin,
};

static serial_t serial = {
    .uart = &huart1,
};

static robot_t robot = {
    .state = ROBOT_STATE_MANUAL,
    .hog[0].motor = &motor1,
    .hog[1].motor = &motor2,
    .imu = &imu,
    .flow = &flow,
};

static reference_t reference = {
    .type = REFERENCE_TYPE_NONE
};

void HAL_TIMEx_CommutCallback(TIM_HandleTypeDef *htim) {
	motor_commutation_callback(&motor1, htim);
    motor_commutation_callback(&motor2, htim);
}

void HAL_GPIO_EXTI_Rising_Callback(uint16_t GPIO_Pin) {
	motor_interrupt_callback(&motor1, GPIO_Pin);
    motor_interrupt_callback(&motor2, GPIO_Pin);
    imu_interrupt_callback(&imu, GPIO_Pin);
}

void HAL_GPIO_EXTI_Falling_Callback(uint16_t GPIO_Pin) {
	motor_interrupt_callback(&motor1, GPIO_Pin);
    motor_interrupt_callback(&motor2, GPIO_Pin);
}

void HAL_I2C_MemRxCpltCallback(I2C_HandleTypeDef *hi2c) {
    imu_received_callback(&imu, hi2c);
}

void HAL_SPI_TxRxCpltCallback(SPI_HandleTypeDef *hspi) {
    flow_transmit_callback(&flow, hspi);
}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart) {
    serial_transmit_callback(&serial, huart);
}

void app_main() {

    motor_init(&motor1);
    motor_init(&motor2);
    serial_init(&serial);
    imu_init(&imu);
    flow_init(&flow);

    uint32_t task_state = 0;

    uint8_t msgpack[10*1024];
    uint8_t decoder_payload[10*1024];
    lrcp_decoder_t decoder;
    lrcp_frame_decoder_init(&decoder);

    while(1) {
        robot.time = HAL_GetTick();

        if((robot.time - task_state)>=20) {
            task_state = robot.time;

            HAL_ADC_Start(&hadc1);
            HAL_ADC_PollForConversion(&hadc1, 2);
            const uint32_t supply_raw = HAL_ADC_GetValue(&hadc1);
            robot.supply = (6.1f*3.3f*supply_raw)/((float)(1<<14));

            const uint32_t size = serialize_robot(&robot, msgpack, sizeof(msgpack));

            lrcp_frame_encode(&serial.base, msgpack, size);
        }

        {
            const uint32_t size = lrcp_frame_decode(&serial.base, &decoder, decoder_payload, sizeof(decoder_payload));

            if(size==1) {
                HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, *(uint8_t *)decoder_payload);
            } else if(size>0) {
                if(deserialize_reference(decoder_payload, size, &reference)) {
                    // TODO
                }
            }
        }

        motor_tick(&motor1);
        motor_tick(&motor2);
        serial_tick(&serial);
        imu_tick(&imu);
        flow_tick(&flow);
    }
}
