/*
 * mpu6050.c
 *
 *  Created on: 08.10.2018
 *  	License: MIT
 *      Author: Mateusz Salamon
 *      Based on:
 *      	 - MPU-6000 and MPU-6050 Product Specification Revision 3.4
 *      	 - MPU-6000 and MPU-6050 Register Map and Descriptions Revision 4.2
 *      	 - i2cdevlib by Jeff Rowberg on MIT license
 *      	 - SparkFun MPU-9250 Digital Motion Processor (DMP) Arduino Library on MIT License
 *
 *		www.msalamon.pl
 *		mateusz@msalamon.pl
 *
 *	Website: https://msalamon.pl/6-stopni-swobody-z-mpu6050-na-stm32/
 *	GitHub: https://github.com/lamik/MPU6050_STM32_HAL
 */

#include "stm32f4xx_hal.h"
#include "i2c.h"

#include "mpu6050.h"
#include "math.h"

#define I2C_TIMEOUT 10

I2C_HandleTypeDef *i2c;
float Acc_Scale;
float Gyr_Scale;

//
// CONFIG
//
void MPU6050_SetDlpf(uint8_t Value)
{
	uint8_t tmp;
	HAL_I2C_Mem_Read(i2c, MPU6050_ADDRESS, MPU6050_RA_CONFIG, 1, &tmp, 1, I2C_TIMEOUT);
	tmp &= 0xF8;
	tmp |= (Value & 0x7);
	HAL_I2C_Mem_Write(i2c, MPU6050_ADDRESS, MPU6050_RA_CONFIG, 1, &tmp, 1, I2C_TIMEOUT);
}

//
// PWR_MGMT_1
//
void MPU6050_DeviceReset(uint8_t Reset)
{
	uint8_t tmp;
	HAL_I2C_Mem_Read(i2c, MPU6050_ADDRESS, MPU6050_RA_PWR_MGMT_1, 1, &tmp, 1, I2C_TIMEOUT);
	tmp &= ~(1<<MPU6050_PWR1_DEVICE_RESET_BIT);
	tmp |= ((Reset & 0x1) << MPU6050_PWR1_DEVICE_RESET_BIT);
	HAL_I2C_Mem_Write(i2c, MPU6050_ADDRESS, MPU6050_RA_PWR_MGMT_1, 1, &tmp, 1, I2C_TIMEOUT);
}

void MPU6050_SetSleepEnabled(uint8_t Enable)
{
	uint8_t tmp;
	HAL_I2C_Mem_Read(i2c, MPU6050_ADDRESS, MPU6050_RA_PWR_MGMT_1, 1, &tmp, 1, I2C_TIMEOUT);
	tmp &= ~(1<<MPU6050_PWR1_SLEEP_BIT);
	tmp |= ((Enable & 0x1) << MPU6050_PWR1_SLEEP_BIT);
	HAL_I2C_Mem_Write(i2c, MPU6050_ADDRESS, MPU6050_RA_PWR_MGMT_1, 1, &tmp, 1, I2C_TIMEOUT);
}

void MPU6050_SetCycleEnabled(uint8_t Enable)
{
	uint8_t tmp;
	HAL_I2C_Mem_Read(i2c, MPU6050_ADDRESS, MPU6050_RA_PWR_MGMT_1, 1, &tmp, 1, I2C_TIMEOUT);
	tmp &= ~(1<<MPU6050_PWR1_CYCLE_BIT);
	tmp |= ((Enable & 0x1) << MPU6050_PWR1_CYCLE_BIT);
	HAL_I2C_Mem_Write(i2c, MPU6050_ADDRESS, MPU6050_RA_PWR_MGMT_1, 1, &tmp, 1, I2C_TIMEOUT);
}

void MPU6050_SetTemperatureSensorDisbled(uint8_t Disable)
{
	uint8_t tmp;
	HAL_I2C_Mem_Read(i2c, MPU6050_ADDRESS, MPU6050_RA_PWR_MGMT_1, 1, &tmp, 1, I2C_TIMEOUT);
	tmp &= ~(1<<MPU6050_PWR1_TEMP_DIS_BIT);
	tmp |= ((Disable & 0x1) << MPU6050_PWR1_TEMP_DIS_BIT);
	HAL_I2C_Mem_Write(i2c, MPU6050_ADDRESS, MPU6050_RA_PWR_MGMT_1, 1, &tmp, 1, I2C_TIMEOUT);
}

void MPU6050_SetClockSource(uint8_t Source)
{
	uint8_t tmp;
	HAL_I2C_Mem_Read(i2c, MPU6050_ADDRESS, MPU6050_RA_PWR_MGMT_1, 1, &tmp, 1, I2C_TIMEOUT);
	tmp &= 0xF8;
	tmp |= (Source & 0x7);
	HAL_I2C_Mem_Write(i2c, MPU6050_ADDRESS, MPU6050_RA_PWR_MGMT_1, 1, &tmp, 1, I2C_TIMEOUT);
}

//
//	PWR_MGMT_2
//
void MPU6050_SetLowPowerWakeUpFrequency(uint8_t Frequency)
{
	uint8_t tmp;
	HAL_I2C_Mem_Read(i2c, MPU6050_ADDRESS, MPU6050_RA_PWR_MGMT_2, 1, &tmp, 1, I2C_TIMEOUT);
	tmp &= 0x3F;
	tmp |= (Frequency & 0x3) << MPU6050_PWR2_LP_WAKE_CTRL_BIT;
	HAL_I2C_Mem_Write(i2c, MPU6050_ADDRESS, MPU6050_RA_PWR_MGMT_2, 1, &tmp, 1, I2C_TIMEOUT);
}

void MPU6050_AccelerometerAxisStandby(uint8_t XA_Stby, uint8_t YA_Stby, uint8_t ZA_Stby)
{
	uint8_t tmp;
	HAL_I2C_Mem_Read(i2c, MPU6050_ADDRESS, MPU6050_RA_PWR_MGMT_2, 1, &tmp, 1, I2C_TIMEOUT);
	tmp &= 0xC7;
	tmp |= ((XA_Stby&0x1) << MPU6050_PWR2_STBY_XA_BIT)|((YA_Stby&0x1) << MPU6050_PWR2_STBY_YA_BIT)|((ZA_Stby&0x1) << MPU6050_PWR2_STBY_ZA_BIT) ;
	HAL_I2C_Mem_Write(i2c, MPU6050_ADDRESS, MPU6050_RA_PWR_MGMT_2, 1, &tmp, 1, I2C_TIMEOUT);
}

void MPU6050_GyroscopeAxisStandby(uint8_t XG_Stby, uint8_t YG_Stby, uint8_t ZG_Stby)
{
	uint8_t tmp;
	HAL_I2C_Mem_Read(i2c, MPU6050_ADDRESS, MPU6050_RA_PWR_MGMT_2, 1, &tmp, 1, I2C_TIMEOUT);
	tmp &= 0xF8;
	tmp |= ((XG_Stby&0x1) << MPU6050_PWR2_STBY_XG_BIT)|((YG_Stby&0x1) << MPU6050_PWR2_STBY_YG_BIT)|((ZG_Stby&0x1) << MPU6050_PWR2_STBY_ZG_BIT) ;
	HAL_I2C_Mem_Write(i2c, MPU6050_ADDRESS, MPU6050_RA_PWR_MGMT_2, 1, &tmp, 1, I2C_TIMEOUT);
}

//
//	Measurement scale configuration
//
void MPU6050_SetFullScaleGyroRange(uint8_t Range)
{
	uint8_t tmp;
	HAL_I2C_Mem_Read(i2c, MPU6050_ADDRESS, MPU6050_RA_GYRO_CONFIG, 1, &tmp, 1, I2C_TIMEOUT);
	tmp &= 0xE7;
	tmp |= ((Range & 0x7) << 3);
	HAL_I2C_Mem_Write(i2c, MPU6050_ADDRESS, MPU6050_RA_GYRO_CONFIG, 1, &tmp, 1, I2C_TIMEOUT);

	switch(Range)
	{
		case MPU6050_GYRO_FS_250:
			Gyr_Scale = 0.007633;
			break;
		case MPU6050_GYRO_FS_500:
			Gyr_Scale = 0.015267;
			break;
		case MPU6050_GYRO_FS_1000:
			Gyr_Scale = 0.030487;
			break;
		case MPU6050_GYRO_FS_2000:
			Gyr_Scale = 0.060975;
			break;
		default:
			break;
	}
}

void MPU6050_SetFullScaleAccelRange(uint8_t Range)
{
	uint8_t tmp;
	HAL_I2C_Mem_Read(i2c, MPU6050_ADDRESS, MPU6050_RA_ACCEL_CONFIG, 1, &tmp, 1, I2C_TIMEOUT);
	tmp &= 0xE7;
	tmp |= ((Range & 0x7) << 3);
	HAL_I2C_Mem_Write(i2c, MPU6050_ADDRESS, MPU6050_RA_ACCEL_CONFIG, 1, &tmp, 1, I2C_TIMEOUT);

	switch(Range)
	{
		case MPU6050_ACCEL_FS_2:
			Acc_Scale = 0.000061;
			break;
		case MPU6050_ACCEL_FS_4:
			Acc_Scale = 0.000122;
			break;
		case MPU6050_ACCEL_FS_8:
			Acc_Scale = 0.000244;
			break;
		case MPU6050_ACCEL_FS_16:
			Acc_Scale = 0.0004882;
			break;
		default:
			break;
	}
}

//
// Reading data
//
int16_t MPU6050_GetTemperatureRAW(void)
{
	uint8_t tmp[2];
	HAL_I2C_Mem_Read(i2c, MPU6050_ADDRESS, MPU6050_RA_TEMP_OUT_H, 1, tmp, 2, I2C_TIMEOUT);
	return (((int16_t)tmp[0]) << 8) | tmp[1];
}

float MPU6050_GetTemperatureCelsius(void)
{
	int16_t temp;

	temp = MPU6050_GetTemperatureRAW();

	return (float)temp / 340 + 36.53;
}

int16_t MPU6050_GetAccelerationXRAW(void)
{
	uint8_t tmp[2];
	HAL_I2C_Mem_Read(i2c, MPU6050_ADDRESS, MPU6050_RA_ACCEL_XOUT_H, 1, tmp, 2, I2C_TIMEOUT);
	return (((int16_t)tmp[0]) << 8) | tmp[1];
}

int16_t MPU6050_GetAccelerationYRAW(void)
{
	uint8_t tmp[2];
	HAL_I2C_Mem_Read(i2c, MPU6050_ADDRESS, MPU6050_RA_ACCEL_YOUT_H, 1, tmp, 2, I2C_TIMEOUT);
	return (((int16_t)tmp[0]) << 8) | tmp[1];
}

int16_t MPU6050_GetAccelerationZRAW(void)
{
	uint8_t tmp[2];
	HAL_I2C_Mem_Read(i2c, MPU6050_ADDRESS, MPU6050_RA_ACCEL_ZOUT_H, 1, tmp, 2, I2C_TIMEOUT);
	return (((int16_t)tmp[0]) << 8) | tmp[1];
}

void MPU6050_GetAccelerometerRAW(int16_t *x, int16_t *y, int16_t *z)
{
	uint8_t tmp[6];
	HAL_I2C_Mem_Read(i2c, MPU6050_ADDRESS, MPU6050_RA_ACCEL_XOUT_H, 1, tmp, 6, I2C_TIMEOUT);

	*x = (((int16_t)tmp[0]) << 8) | tmp[1];
	*y = (((int16_t)tmp[2]) << 8) | tmp[3];
	*z = (((int16_t)tmp[4]) << 8) | tmp[5];
}

void MPU6050_GetAccelerometerScaled(float* x, float* y, float* z)
{
	int16_t tmp_x, tmp_y, tmp_z;
	MPU6050_GetAccelerometerRAW(&tmp_x, &tmp_y, &tmp_z);

	*x = (float)tmp_x * Acc_Scale;
	*y = (float)tmp_y * Acc_Scale;
	*z = (float)tmp_z * Acc_Scale;
}

int16_t MPU6050_GetRotationXRAW(void)
{
	uint8_t tmp[2];
	HAL_I2C_Mem_Read(i2c, MPU6050_ADDRESS, MPU6050_RA_GYRO_XOUT_H, 1, tmp, 2, I2C_TIMEOUT);
	return (((int16_t)tmp[0]) << 8) | tmp[1];
}

int16_t MPU6050_GetRotationYRAW(void)
{
	uint8_t tmp[2];
	HAL_I2C_Mem_Read(i2c, MPU6050_ADDRESS, MPU6050_RA_GYRO_YOUT_H, 1, tmp, 2, I2C_TIMEOUT);
	return (((int16_t)tmp[0]) << 8) | tmp[1];
}

int16_t MPU6050_GetRotationZRAW(void)
{
	uint8_t tmp[2];
	HAL_I2C_Mem_Read(i2c, MPU6050_ADDRESS, MPU6050_RA_GYRO_ZOUT_H, 1, tmp, 2, I2C_TIMEOUT);
	return (((int16_t)tmp[0]) << 8) | tmp[1];
}

void MPU6050_GetGyroscopeRAW(int16_t *x, int16_t *y, int16_t *z)
{
	uint8_t tmp[6];
	HAL_I2C_Mem_Read(i2c, MPU6050_ADDRESS, MPU6050_RA_GYRO_XOUT_H, 1, tmp, 6, I2C_TIMEOUT);

	*x = (((int16_t)tmp[0]) << 8) | tmp[1];
	*y = (((int16_t)tmp[2]) << 8) | tmp[3];
	*z = (((int16_t)tmp[4]) << 8) | tmp[5];
}

void MPU6050_GetGyroscopeScaled(float* x, float* y, float* z)
{
	int16_t tmp_x, tmp_y, tmp_z;

	MPU6050_GetGyroscopeRAW(&tmp_x, &tmp_y, &tmp_z);

	*x = (float)tmp_x * Gyr_Scale;
	*y = (float)tmp_y * Gyr_Scale;
	*z = (float)tmp_z * Gyr_Scale;
}

void MPU6050_GetRollPitch(float* Roll, float* Pitch)
{
	float acc_x, acc_y, acc_z;
	MPU6050_GetAccelerometerScaled(&acc_x, &acc_y, &acc_z);

	*Roll = atan2(acc_y, acc_z) * 180.0 / M_PI;
	*Pitch = -(atan2(acc_x, sqrt(acc_y*acc_y + acc_z*acc_z))*180.0)/M_PI;
}

//
//	Setting INT pin
//
void MPU6050_SetInterruptMode(uint8_t Mode)
{
	uint8_t tmp;
	HAL_I2C_Mem_Read(i2c, MPU6050_ADDRESS, MPU6050_RA_INT_PIN_CFG, 1, &tmp, 1, I2C_TIMEOUT);
	tmp &= ~(1 << MPU6050_INTCFG_INT_LEVEL_BIT);
	tmp |= ((Mode & 0x1) << MPU6050_INTCFG_INT_LEVEL_BIT);
	HAL_I2C_Mem_Write(i2c, MPU6050_ADDRESS, MPU6050_RA_INT_PIN_CFG, 1, &tmp, 1, I2C_TIMEOUT);
}

void MPU6050_SetInterruptDrive(uint8_t Drive)
{
	uint8_t tmp;
	HAL_I2C_Mem_Read(i2c, MPU6050_ADDRESS, MPU6050_RA_INT_PIN_CFG, 1, &tmp, 1, I2C_TIMEOUT);
	tmp &= ~(1 << MPU6050_INTCFG_INT_OPEN_BIT);
	tmp |= ((Drive & 0x1) << MPU6050_INTCFG_INT_OPEN_BIT);
	HAL_I2C_Mem_Write(i2c, MPU6050_ADDRESS, MPU6050_RA_INT_PIN_CFG, 1, &tmp, 1, I2C_TIMEOUT);
}

void MPU6050_SetInterruptLatch(uint8_t Latch)
{
	uint8_t tmp;
	HAL_I2C_Mem_Read(i2c, MPU6050_ADDRESS, MPU6050_RA_INT_PIN_CFG, 1, &tmp, 1, I2C_TIMEOUT);
	tmp &= ~(1 << MPU6050_INTCFG_INT_RD_CLEAR_BIT);
	tmp |= ((Latch & 0x1) << MPU6050_INTCFG_INT_RD_CLEAR_BIT);
	HAL_I2C_Mem_Write(i2c, MPU6050_ADDRESS, MPU6050_RA_INT_PIN_CFG, 1, &tmp, 1, I2C_TIMEOUT);
}

void MPU6050_SetInterruptLatchClear(uint8_t Clear)
{
	uint8_t tmp;
	HAL_I2C_Mem_Read(i2c, MPU6050_ADDRESS, MPU6050_RA_INT_PIN_CFG, 1, &tmp, 1, I2C_TIMEOUT);
	tmp &= ~(1 << MPU6050_INTCFG_LATCH_INT_EN_BIT);
	tmp |= ((Clear & 0x1) << MPU6050_INTCFG_LATCH_INT_EN_BIT);
	HAL_I2C_Mem_Write(i2c, MPU6050_ADDRESS, MPU6050_RA_INT_PIN_CFG, 1, &tmp, 1, I2C_TIMEOUT);
}

void MPU6050_SetIntEnableRegister(uint8_t Value)
{
	HAL_I2C_Mem_Write(i2c, MPU6050_ADDRESS, MPU6050_RA_INT_ENABLE, 1, &Value, 1, I2C_TIMEOUT);
}

void MPU6050_SetIntDataReadyEnabled(uint8_t Enable)
{
	uint8_t tmp;
	HAL_I2C_Mem_Read(i2c, MPU6050_ADDRESS, MPU6050_RA_INT_ENABLE, 1, &tmp, 1, I2C_TIMEOUT);
	tmp &= ~(1 << MPU6050_INTERRUPT_DATA_RDY_BIT);
	tmp |= ((Enable & 0x1) << MPU6050_INTERRUPT_DATA_RDY_BIT);
	HAL_I2C_Mem_Write(i2c, MPU6050_ADDRESS, MPU6050_RA_INT_ENABLE, 1, &tmp, 1, I2C_TIMEOUT);
}

uint8_t MPU6050_GetIntStatusRegister(void)
{
	uint8_t tmp;
	HAL_I2C_Mem_Read(i2c, MPU6050_ADDRESS, MPU6050_RA_INT_STATUS, 1, &tmp, 1, I2C_TIMEOUT);
	return tmp;
}

uint8_t MPU6050_GetDeviceID(void)
{
	uint8_t tmp;
	HAL_I2C_Mem_Read(i2c, MPU6050_ADDRESS, MPU6050_RA_WHO_AM_I, 1, &tmp, 1, I2C_TIMEOUT);
	return tmp<<1;
}

//
//	Motion functions - not included in documentation/register map
//
void MPU6050_SetDHPFMode(uint8_t Dhpf)
{
	uint8_t tmp;
	HAL_I2C_Mem_Read(i2c, MPU6050_ADDRESS, MPU6050_RA_ACCEL_CONFIG, 1, &tmp, 1, I2C_TIMEOUT);
	tmp &= ~(0x07);
	tmp |= Dhpf & 0x7;
	HAL_I2C_Mem_Write(i2c, MPU6050_ADDRESS, MPU6050_RA_ACCEL_CONFIG, 1, &tmp, 1, I2C_TIMEOUT);
}

uint8_t MPU6050_GetMotionStatusRegister(void)
{
	uint8_t tmp;
	HAL_I2C_Mem_Read(i2c, MPU6050_ADDRESS, MPU6050_RA_MOT_DETECT_STATUS, 1, &tmp, 1, I2C_TIMEOUT);
	return tmp;
}

void MPU6050_SetIntZeroMotionEnabled(uint8_t Enable)
{
	uint8_t tmp;
	HAL_I2C_Mem_Read(i2c, MPU6050_ADDRESS, MPU6050_RA_INT_ENABLE, 1, &tmp, 1, I2C_TIMEOUT);
	tmp &= ~(1 << MPU6050_INTERRUPT_ZMOT_BIT);
	tmp |= ((Enable & 0x1) << MPU6050_INTERRUPT_ZMOT_BIT);
	HAL_I2C_Mem_Write(i2c, MPU6050_ADDRESS, MPU6050_RA_INT_ENABLE, 1, &tmp, 1, I2C_TIMEOUT);
}

void MPU6050_SetIntMotionEnabled(uint8_t Enable)
{
	uint8_t tmp;
	HAL_I2C_Mem_Read(i2c, MPU6050_ADDRESS, MPU6050_RA_INT_ENABLE, 1, &tmp, 1, I2C_TIMEOUT);
	tmp &= ~(1 << MPU6050_INTERRUPT_MOT_BIT);
	tmp |= ((Enable & 0x1) << MPU6050_INTERRUPT_MOT_BIT);
	HAL_I2C_Mem_Write(i2c, MPU6050_ADDRESS, MPU6050_RA_INT_ENABLE, 1, &tmp, 1, I2C_TIMEOUT);
}

void MPU6050_SetIntFreeFallEnabled(uint8_t Enable)
{
	uint8_t tmp;
	HAL_I2C_Mem_Read(i2c, MPU6050_ADDRESS, MPU6050_RA_INT_ENABLE, 1, &tmp, 1, I2C_TIMEOUT);
	tmp &= ~(1 << MPU6050_INTERRUPT_FF_BIT);
	tmp |= ((Enable & 0x1) << MPU6050_INTERRUPT_FF_BIT);
	HAL_I2C_Mem_Write(i2c, MPU6050_ADDRESS, MPU6050_RA_INT_ENABLE, 1, &tmp, 1, I2C_TIMEOUT);
}

void MPU6050_SetMotionDetectionThreshold(uint8_t Threshold)
{
	HAL_I2C_Mem_Write(i2c, MPU6050_ADDRESS, MPU6050_RA_MOT_THR, 1, &Threshold, 1, I2C_TIMEOUT);
}

void MPU6050_SetMotionDetectionDuration(uint8_t Duration)
{
	HAL_I2C_Mem_Write(i2c, MPU6050_ADDRESS, MPU6050_RA_MOT_DUR, 1, &Duration, 1, I2C_TIMEOUT);
}

void MPU6050_SetZeroMotionDetectionThreshold(uint8_t Threshold)
{
	HAL_I2C_Mem_Write(i2c, MPU6050_ADDRESS, MPU6050_RA_ZRMOT_THR, 1, &Threshold, 1, I2C_TIMEOUT);
}

void MPU6050_SetZeroMotionDetectionDuration(uint8_t Duration)
{
	HAL_I2C_Mem_Write(i2c, MPU6050_ADDRESS, MPU6050_RA_ZRMOT_DUR, 1, &Duration, 1, I2C_TIMEOUT);
}

void MPU6050_SetFreeFallDetectionThreshold(uint8_t Threshold)
{
	HAL_I2C_Mem_Write(i2c, MPU6050_ADDRESS, MPU6050_RA_FF_THR, 1, &Threshold, 1, I2C_TIMEOUT);
}

void MPU6050_SetFreeFallDetectionDuration(uint8_t Duration)
{
	HAL_I2C_Mem_Write(i2c, MPU6050_ADDRESS, MPU6050_RA_FF_DUR, 1, &Duration, 1, I2C_TIMEOUT);
}

//
//	Initialization
//
void MPU6050_Init(I2C_HandleTypeDef *hi2c)
{
	i2c = hi2c;
	MPU6050_DeviceReset(1);
    MPU6050_SetSleepEnabled(0);
    MPU6050_SetClockSource(MPU6050_CLOCK_INTERNAL);
    MPU6050_SetDlpf(MPU6050_DLPF_BW_20);
    MPU6050_SetFullScaleGyroRange(MPU6050_GYRO_FS_250);
    MPU6050_SetFullScaleAccelRange(MPU6050_ACCEL_FS_2);

}
