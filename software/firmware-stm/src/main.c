#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stm32u5xx_hal.h>
#include <cmp/cmp.h>
#include <lrcp/frame.h>

#include "main.h"
#include "motor.h"
#include "mpu6050_regs.h"
#include "pmw3901_regs.h"
#include "serial.h"

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

static volatile bool imu_ready = false;
static uint8_t imu_buffer[14];
static float imu_accel[3] = {0};
static float imu_gyro[3] = {0};

static volatile bool flow_ready = false;
static uint8_t flow_buffer_tx[32];
static uint8_t flow_buffer_rx[32];
static float flow_vel[2] = {0};

typedef struct {
    uint8_t *buffer;
    size_t capacity;
    size_t position;
    size_t size;
} buffer_t;

void HAL_TIMEx_CommutCallback(TIM_HandleTypeDef *htim) {
	motor_commutation_callback(&motor1, htim);
    motor_commutation_callback(&motor2, htim);
}

void HAL_GPIO_EXTI_Rising_Callback(uint16_t GPIO_Pin) {
	motor_interrupt_callback(&motor1, GPIO_Pin);
    motor_interrupt_callback(&motor2, GPIO_Pin);

    if(GPIO_Pin==IMU_INT_Pin) {
		HAL_I2C_Mem_Read_DMA(&hi2c1, MPU6050_ADDR<<1, MPU6050_REG_ACCEL_XOUT_H, 1, imu_buffer, sizeof(imu_buffer));
	}
}

void HAL_GPIO_EXTI_Falling_Callback(uint16_t GPIO_Pin) {
	motor_interrupt_callback(&motor1, GPIO_Pin);
    motor_interrupt_callback(&motor2, GPIO_Pin);
}

void HAL_I2C_MemRxCpltCallback(I2C_HandleTypeDef *hi2c) {
	if(hi2c==&hi2c1) {
	    imu_ready = true;
	}
}

void HAL_SPI_TxRxCpltCallback(SPI_HandleTypeDef *hspi) {
	if(hspi==&hspi2) {
	    flow_ready = true;
	}
}

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

static void mpu6050_write(uint8_t address, uint8_t value) {
	HAL_I2C_Mem_Write(&hi2c1, MPU6050_ADDR<<1, address, 1, &value, 1, 100);
}

static void mpu6050_init() {
	mpu6050_write(MPU6050_REG_PWR_MGMT_1,
		MPU6050_PWR_MGMT_1_DEVICE_RESET
	);

	HAL_Delay(100);

	mpu6050_write(MPU6050_REG_SIGNAL_PATH_RESET,
		MPU6050_SIGNAL_PATH_RESET_GYRO |
		MPU6050_SIGNAL_PATH_RESET_ACCEL |
		MPU6050_SIGNAL_PATH_RESET_TEMP
	);

	HAL_Delay(100);

	mpu6050_write(MPU6050_REG_INT_ENABLE,
		MPU6050_INT_ENABLE_FIFO_OVERLOW_DISABLE |
		MPU6050_INT_ENABLE_I2C_MST_INT_DISABLE |
		MPU6050_INT_ENABLE_DATA_RDY_ENABLE
	);

	mpu6050_write(MPU6050_REG_INT_PIN_CFG,
		MPU6050_INT_PIN_CFG_LEVEL_ACTIVE_HIGH |
		MPU6050_INT_PIN_CFG_PUSH_PULL |
		MPU6050_INT_PIN_CFG_PULSE |
		MPU6050_INT_PIN_CFG_STATUS_CLEAR_AFTER_ANY |
		MPU6050_INT_PIN_CFG_FSYNC_DISABLE |
		MPU6050_INT_PIN_CFG_I2C_BYPASS_DISABLE
	);

	mpu6050_write(MPU6050_REG_PWR_MGMT_1,
		MPU6050_PWR_MGMT_1_TEMP_DIS |
		MPU6050_PWR_MGMT_1_CLOCK_INTERNAL
	);

	mpu6050_write(MPU6050_REG_CONFIG,
		MPU6050_CONFIG_EXT_SYNC_DISABLED |
		MPU6050_CONFIG_DLPF_SETTING_6
	);

	mpu6050_write(MPU6050_REG_ACCEL_CONFIG,
		MPU6050_ACCEL_CONFIG_RANGE_4G
	);

	mpu6050_write(MPU6050_REG_GYRO_CONFIG,
		MPU6050_GYRO_CONFIG_RANGE_500DPS
	);

	mpu6050_write(MPU6050_REG_SMPLRT_DIV, 0);
}

static void mpu6050_read(float *acc, float *gyr, const uint8_t *buffer) {
	{
		const int16_t raw_x = (((int16_t)buffer[8])<<8) | buffer[9];
		const int16_t raw_y = (((int16_t)buffer[10])<<8) | buffer[11];
		const int16_t raw_z = (((int16_t)buffer[12])<<8) | buffer[13];

		const float gain = 65.5f;
		const float dps_to_rads = 0.017453292519943f;

		gyr[0] = -raw_x*dps_to_rads/gain;
		gyr[1] = -raw_y*dps_to_rads/gain;
		gyr[2] = +raw_z*dps_to_rads/gain;
	}

	{
		const int16_t raw_x = (((int16_t)buffer[0])<<8) | buffer[1];
		const int16_t raw_y = (((int16_t)buffer[2])<<8) | buffer[3];
		const int16_t raw_z = (((int16_t)buffer[4])<<8) | buffer[5];

		const float gain = 8192.f;
		const float g_to_ms2 = 9.80665f;

		acc[0] = -raw_x*g_to_ms2/gain;
		acc[1] = -raw_y*g_to_ms2/gain;
		acc[2] = +raw_z*g_to_ms2/gain;
	}
}

void pmw3901_write(const uint8_t address, const uint8_t value) {
	uint8_t tx[] = {address | 0x80, value};

	HAL_GPIO_WritePin(FLOW_CS_GPIO_Port, FLOW_CS_Pin, GPIO_PIN_RESET);
	HAL_Delay(1);
	HAL_SPI_Transmit(&hspi2, tx, sizeof(tx), HAL_MAX_DELAY);
	HAL_Delay(1);
	HAL_GPIO_WritePin(FLOW_CS_GPIO_Port, FLOW_CS_Pin, GPIO_PIN_SET);
	HAL_Delay(1);
}

void pmw3901_init() {
	pmw3901_write(0x3A, 0x5A);

	HAL_Delay(5);

	pmw3901_write(0x7F, 0x00);
	pmw3901_write(0x61, 0xAD);
	pmw3901_write(0x7F, 0x03);
	pmw3901_write(0x40, 0x00);
	pmw3901_write(0x7F, 0x05);
	pmw3901_write(0x41, 0xB3);
	pmw3901_write(0x43, 0xF1);
	pmw3901_write(0x45, 0x14);
	pmw3901_write(0x5B, 0x32);
	pmw3901_write(0x5F, 0x34);
	pmw3901_write(0x7B, 0x08);
	pmw3901_write(0x7F, 0x06);
	pmw3901_write(0x44, 0x1B);
	pmw3901_write(0x40, 0xBF);
	pmw3901_write(0x4E, 0x3F);
	pmw3901_write(0x7F, 0x08);
	pmw3901_write(0x65, 0x20);
	pmw3901_write(0x6A, 0x18);
	pmw3901_write(0x7F, 0x09);
	pmw3901_write(0x4F, 0xAF);
	pmw3901_write(0x5F, 0x40);
	pmw3901_write(0x48, 0x80);
	pmw3901_write(0x49, 0x80);
	pmw3901_write(0x57, 0x77);
	pmw3901_write(0x60, 0x78);
	pmw3901_write(0x61, 0x78);
	pmw3901_write(0x62, 0x08);
	pmw3901_write(0x63, 0x50);
	pmw3901_write(0x7F, 0x0A);
	pmw3901_write(0x45, 0x60);
	pmw3901_write(0x7F, 0x00);
	pmw3901_write(0x4D, 0x11);
	pmw3901_write(0x55, 0x80);
	pmw3901_write(0x74, 0x1F);
	pmw3901_write(0x75, 0x1F);
	pmw3901_write(0x4A, 0x78);
	pmw3901_write(0x4B, 0x78);
	pmw3901_write(0x44, 0x08);
	pmw3901_write(0x45, 0x50);
	pmw3901_write(0x64, 0xFF);
	pmw3901_write(0x65, 0x1F);
	pmw3901_write(0x7F, 0x14);
	pmw3901_write(0x65, 0x60);
	pmw3901_write(0x66, 0x08);
	pmw3901_write(0x63, 0x78);
	pmw3901_write(0x7F, 0x15);
	pmw3901_write(0x48, 0x58);
	pmw3901_write(0x7F, 0x07);
	pmw3901_write(0x41, 0x0D);
	pmw3901_write(0x43, 0x14);
	pmw3901_write(0x4B, 0x0E);
	pmw3901_write(0x45, 0x0F);
	pmw3901_write(0x44, 0x42);
	pmw3901_write(0x4C, 0x80);
	pmw3901_write(0x7F, 0x10);
	pmw3901_write(0x5B, 0x02);
	pmw3901_write(0x7F, 0x07);
	pmw3901_write(0x40, 0x41);
	pmw3901_write(0x70, 0x00);

	HAL_Delay(100);

	pmw3901_write(0x32, 0x44);
	pmw3901_write(0x7F, 0x07);
	pmw3901_write(0x40, 0x40);
	pmw3901_write(0x7F, 0x06);
	pmw3901_write(0x62, 0xf0);
	pmw3901_write(0x63, 0x00);
	pmw3901_write(0x7F, 0x0D);
	pmw3901_write(0x48, 0xC0);
	pmw3901_write(0x6F, 0xd5);
	pmw3901_write(0x7F, 0x00);
	pmw3901_write(0x5B, 0xa0);
	pmw3901_write(0x4E, 0xA8);
	pmw3901_write(0x5A, 0x50);
	pmw3901_write(0x40, 0x80);
}

void pmw3901_read(float *flow, const uint8_t *buffer, const float dt) {
	const int16_t delta_x = (((int16_t)buffer[2])<<8) | buffer[1];
	const int16_t delta_y = (((int16_t)buffer[4])<<8) | buffer[3];

	flow[0] = -delta_y/(dt*PMW3901_FOCAL_LENGTH);
	flow[1] = -delta_x/(dt*PMW3901_FOCAL_LENGTH);
}

void app_main() {
    serial_t serial;
    serial_init(&serial, &huart1);

    motor_init(&motor1);
    motor_init(&motor2);

    mpu6050_init();
    pmw3901_init();

    uint32_t task_state = 0;
    uint32_t task_flow = 0;

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

    uint8_t msgpack[10*1024];
    uint8_t decoder_payload[10*1024];
    lrcp_decoder_t decoder;
    lrcp_frame_decoder_init(&decoder);

    while(1) {
        const uint32_t time = HAL_GetTick();

        if(imu_ready) {
		    imu_ready = false;
		    mpu6050_read(imu_accel, imu_gyro, imu_buffer);
	    }

        if((time - task_flow)>=20) {
            task_flow = time;
            for(uint8_t i=0; i<5; i++) {
                flow_buffer_tx[2*i] = (PMW3901_REG_MOTION + i) & ~0x80;
            }
            HAL_GPIO_WritePin(FLOW_CS_GPIO_Port, FLOW_CS_Pin, GPIO_PIN_RESET);
            HAL_SPI_TransmitReceive_IT(&hspi2, flow_buffer_tx, flow_buffer_rx, 10);
        }

        if(flow_ready) {
            flow_ready = false;
            HAL_GPIO_WritePin(FLOW_CS_GPIO_Port, FLOW_CS_Pin, GPIO_PIN_SET);
            uint8_t motion[5];
            for(uint8_t i=0; i<sizeof(motion); i++) {
                motion[i] = flow_buffer_rx[2*i + 1];
            }

            float tmp[2];
            pmw3901_read(tmp, motion, 0.02f);
            if((fabs(tmp[0])<7.4f) && (fabs(tmp[1])<7.4f)) {
                flow_vel[0] = tmp[0];
                flow_vel[1] = tmp[1];
            }
        }

        if((time - task_state)>=20) {
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

            cmp_write_map(&cmp, 6);

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
            cmp_write_float(&cmp, motor1.vel);
            cmp_write_str(&cmp, "state", 5);
            cmp_write_str(&cmp, motor_state[motor1.state], strlen(motor_state[motor1.state]));

            cmp_write_str(&cmp, "hog2", 4);
            cmp_write_map(&cmp, 4);
            cmp_write_str(&cmp, "servox", 6);
            cmp_write_float(&cmp, 0);
            cmp_write_str(&cmp, "servoy", 6);
            cmp_write_float(&cmp, 0);
            cmp_write_str(&cmp, "motor", 5);
            cmp_write_float(&cmp, motor2.vel);
            cmp_write_str(&cmp, "state", 5);
            cmp_write_str(&cmp, motor_state[motor2.state], strlen(motor_state[motor2.state]));

            cmp_write_str(&cmp, "sensors", 7);
            cmp_write_array(&cmp, 3);
            cmp_write_map(&cmp, 3);
            cmp_write_str(&cmp, "name", 4);
            cmp_write_str(&cmp, "accelerometer", 13);
            cmp_write_str(&cmp, "unit", 4);
            cmp_write_str(&cmp, "m/s^2", 5);
            cmp_write_str(&cmp, "data", 4);
            cmp_write_array(&cmp, 3);
            cmp_write_float(&cmp, imu_accel[0]);
            cmp_write_float(&cmp, imu_accel[1]);
            cmp_write_float(&cmp, imu_accel[2]);
            cmp_write_map(&cmp, 3);
            cmp_write_str(&cmp, "name", 4);
            cmp_write_str(&cmp, "gyroscope", 9);
            cmp_write_str(&cmp, "unit", 4);
            cmp_write_str(&cmp, "rad/s", 5);
            cmp_write_str(&cmp, "data", 4);
            cmp_write_array(&cmp, 3);
            cmp_write_float(&cmp, imu_gyro[0]);
            cmp_write_float(&cmp, imu_gyro[1]);
            cmp_write_float(&cmp, imu_gyro[2]);
            cmp_write_map(&cmp, 3);
            cmp_write_str(&cmp, "name", 4);
            cmp_write_str(&cmp, "optical_flow", 12);
            cmp_write_str(&cmp, "unit", 4);
            cmp_write_str(&cmp, "1/s", 5);
            cmp_write_str(&cmp, "data", 4);
            cmp_write_array(&cmp, 2);
            cmp_write_float(&cmp, flow_vel[0]);
            cmp_write_float(&cmp, flow_vel[1]);

            lrcp_frame_encode(&serial.base, buffer.buffer, buffer.size);
        }

        {
            const uint32_t size = lrcp_frame_decode(&serial.base, &decoder, decoder_payload, sizeof(decoder_payload));

            if(size==1) {
                HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, *(uint8_t *)decoder_payload);
            } else if(size>0) {
                buffer_t buffer = {
                    .buffer = (void *)decoder_payload,
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
        }

        /*if(!HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_2)) {
            motor_set_vel(&motor1, 200);
            motor_set_vel(&motor2, 200);
        }*/

        motor_tick(&motor1);
        motor_tick(&motor2);
        serial_tick(&serial);
    }
}
