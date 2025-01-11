#include <stdint.h>
#include <stm32u5xx_hal.h>

#include "mpu6050_regs.h"
#include "imu.h"

static void mpu6050_write(imu_t *imu, uint8_t address, uint8_t value) {
	HAL_I2C_Mem_Write(imu->i2c, MPU6050_ADDR<<1, address, 1, &value, 1, 100);
}

void imu_init(imu_t *imu) {
	mpu6050_write(imu, MPU6050_REG_PWR_MGMT_1,
		MPU6050_PWR_MGMT_1_DEVICE_RESET
	);

	HAL_Delay(10);

	mpu6050_write(imu, MPU6050_REG_SIGNAL_PATH_RESET,
		MPU6050_SIGNAL_PATH_RESET_GYRO |
		MPU6050_SIGNAL_PATH_RESET_ACCEL |
		MPU6050_SIGNAL_PATH_RESET_TEMP
	);

	HAL_Delay(10);

	mpu6050_write(imu, MPU6050_REG_INT_ENABLE,
		MPU6050_INT_ENABLE_FIFO_OVERLOW_DISABLE |
		MPU6050_INT_ENABLE_I2C_MST_INT_DISABLE |
		MPU6050_INT_ENABLE_DATA_RDY_ENABLE
	);

	mpu6050_write(imu, MPU6050_REG_INT_PIN_CFG,
		MPU6050_INT_PIN_CFG_LEVEL_ACTIVE_HIGH |
		MPU6050_INT_PIN_CFG_PUSH_PULL |
		MPU6050_INT_PIN_CFG_PULSE |
		MPU6050_INT_PIN_CFG_STATUS_CLEAR_AFTER_ANY |
		MPU6050_INT_PIN_CFG_FSYNC_DISABLE |
		MPU6050_INT_PIN_CFG_I2C_BYPASS_DISABLE
	);

	mpu6050_write(imu, MPU6050_REG_PWR_MGMT_1,
		MPU6050_PWR_MGMT_1_TEMP_DIS |
		MPU6050_PWR_MGMT_1_CLOCK_INTERNAL
	);

	mpu6050_write(imu, MPU6050_REG_CONFIG,
		MPU6050_CONFIG_EXT_SYNC_DISABLED |
		MPU6050_CONFIG_DLPF_SETTING_6
	);

	mpu6050_write(imu, MPU6050_REG_ACCEL_CONFIG,
		MPU6050_ACCEL_CONFIG_RANGE_4G
	);

	mpu6050_write(imu, MPU6050_REG_GYRO_CONFIG,
		MPU6050_GYRO_CONFIG_RANGE_500DPS
	);

	mpu6050_write(imu, MPU6050_REG_SMPLRT_DIV, 0);
}

void imu_tick(imu_t *imu) {
	if(imu->ready) {
		imu->ready = 0;

		{
			const int16_t raw_x = (((int16_t)imu->buffer[8])<<8) | imu->buffer[9];
			const int16_t raw_y = (((int16_t)imu->buffer[10])<<8) | imu->buffer[11];
			const int16_t raw_z = (((int16_t)imu->buffer[12])<<8) | imu->buffer[13];

			const float gain = 65.5f;
			const float dps_to_rads = 0.017453292519943f;

			imu->gyration[0] = -raw_x*dps_to_rads/gain;
			imu->gyration[1] = -raw_y*dps_to_rads/gain;
			imu->gyration[2] = +raw_z*dps_to_rads/gain;
		}

		{
			const int16_t raw_x = (((int16_t)imu->buffer[0])<<8) | imu->buffer[1];
			const int16_t raw_y = (((int16_t)imu->buffer[2])<<8) | imu->buffer[3];
			const int16_t raw_z = (((int16_t)imu->buffer[4])<<8) | imu->buffer[5];

			const float gain = 8192.f;
			const float g_to_ms2 = 9.80665f;

			imu->acceleration[0] = -raw_x*g_to_ms2/gain;
			imu->acceleration[1] = -raw_y*g_to_ms2/gain;
			imu->acceleration[2] = +raw_z*g_to_ms2/gain;
		}
	}
}

void imu_interrupt_callback(imu_t *imu, const uint16_t pin) {
	if(pin==imu->interrupt) {
		HAL_I2C_Mem_Read_DMA(imu->i2c, MPU6050_ADDR<<1, MPU6050_REG_ACCEL_XOUT_H, 1, imu->buffer, sizeof(imu->buffer));
	}
}

void imu_received_callback(imu_t *imu, const I2C_HandleTypeDef *hi2c) {
	if(hi2c==imu->i2c) {
		imu->ready = 1;
	}
}
