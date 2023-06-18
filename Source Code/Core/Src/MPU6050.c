/*
 * MPU6050.c
 *
 *  Created on: Apr 29, 2023
 *      Author: esref
 */


/*
 * mpu6050.c
 *
 *  Created on: Feb 6, 2023
 *      Author: esref
 */

#include "mpu6050.h"
#define RAD_TO_DEG        (57.295779513082320876798154814105) // 180/PI
#define MPU6050_ADDR      (0xD0)
#define WHO_AM_I_REG      (0x75)
#define PWR_MGMT_1_REG    (0x6B)
#define SMPLRT_DIV_REG    (0x19)
#define DLPF_CONFIG_REG	  (0x1A)// Low pass filter
#define ACCEL_CONFIG_REG  (0x1C)
#define ACCEL_XOUT_H_REG  (0x3B)
#define INT_ENABLE_REG	  (0x38)
#define TEMP_OUT_H_REG    (0x41)
#define GYRO_CONFIG_REG   (0x1B)
#define GYRO_XOUT_H_REG   (0x43)
#define ACCEL_CONFIG_SENS (4096.0)
#define GYRO_CONFIG_SENS  (65.5)

#define GYRO_X_OFFSET     (123)  //TODO
#define GYRO_Y_OFFSET     (9) 	 //TODO
#define GYRO_Z_OFFSET     (-129) //TODO

#define ACCEL_X_OFFSET     (100) //TODO
#define ACCEL_Y_OFFSET     (-30) //TODO Bu offset değerlerini kullanıcıya bir kereliğine nasıl yaptırıp kodda kaydedebiliriz?
#define ACCEL_Z_OFFSET     (-965)

#define I2C_TIME_OUT       (100)

#define SAFETY_ANGLE	  (20)



void kalmanInit(MPU6050_HandleTypeDef* mpu6050)
{
	mpu6050->kalmanAnglePitch = 0;
	mpu6050->kalmanAngleRoll = 0;
	mpu6050->kalmanAngleUncertaintyRoll = 4;
	mpu6050->kalmanAngleUncertaintyPitch = 4;
}

bool MPU6050_Init(I2C_HandleTypeDef *I2Cx, MPU6050_HandleTypeDef *mpu6050)
{
	mpu6050->hasUnsafeAngle = false;
	uint8_t check;
	uint8_t data;
	//TODO hardware veri okuyamadığında target is not resonding hatası.
	// check device ID WHO_AM_I

	HAL_I2C_Mem_Read(I2Cx, MPU6050_ADDR, WHO_AM_I_REG, 1, &check, 1, I2C_TIME_OUT);

	if (check == 104) // 0x68 will be returned by the sensor if everything goes well
	{
		kalmanInit(mpu6050);
		// power management register 0X6B we should write all 0's to wake the sensor up
		data = 0x00;
		HAL_I2C_Mem_Write(I2Cx, MPU6050_ADDR, PWR_MGMT_1_REG, 1, &data, 1, I2C_TIME_OUT);

		// Set DATA RATE of 1KHz by writing SMPLRT_DIV register. TODO Burası tam anlaşılmalı tez için...
		data = 0x07;
		HAL_I2C_Mem_Write(I2Cx, MPU6050_ADDR, SMPLRT_DIV_REG, 1, &data, 1, I2C_TIME_OUT);

		// Set low pass filter.
		data = 0x05;	// Accelerometer Bandwidth (10 Hz), Delay (13.8 ms)
					    // Gyroscope Bandwidth (10 Hz), Delay (13.4 ms), Fs (1kHz)
		HAL_I2C_Mem_Write(I2Cx, MPU6050_ADDR, DLPF_CONFIG_REG, 1, &data, 1, I2C_TIME_OUT);

		data = 0x01; 	// DATA_RDY_EN set to 1.
		HAL_I2C_Mem_Write(I2Cx, MPU6050_ADDR, INT_ENABLE_REG, 1, &data, 1, I2C_TIME_OUT);
		// Set accelerometer configuration in ACCEL_CONFIG Register
		// XA_ST=0,YA_ST=0,ZA_ST=0, FS_SEL=3 -> � 8g
		data = 0x10;
		HAL_I2C_Mem_Write(I2Cx, MPU6050_ADDR, ACCEL_CONFIG_REG, 1, &data, 1, I2C_TIME_OUT);

		// Set Gyro configuration in GYRO_CONFIG Register
		// XG_ST=0,YG_ST=0,ZG_ST=0, FS_SEL=1 -> � 500 �/s
		data = 0x08;
		HAL_I2C_Mem_Write(I2Cx, MPU6050_ADDR, GYRO_CONFIG_REG, 1, &data, 1, I2C_TIME_OUT);
		return 0;
	}
	return 1;
}

bool readTemp(I2C_HandleTypeDef *I2Cx, MPU6050_HandleTypeDef *mpu6050) {
	HAL_StatusTypeDef retval;
	uint8_t recData[2];
	int16_t temp;

	retval = HAL_I2C_Mem_Read(I2Cx, MPU6050_ADDR, TEMP_OUT_H_REG, 1, recData, 2, I2C_TIME_OUT);

	// Read 2 BYTES of data starting from TEMP_OUT_H_REG register
	if(HAL_OK == retval)
	{
		temp = (int16_t) (recData[0] << 8 | recData[1]);
		mpu6050->temperature = (float) ((int16_t) temp / (float) 340.0 + (float) 36.53);
		return 0;
	}
	else
	{
		return 1;
	}

}

// sets angles according to accelerometer values if reading process is done successfully
bool readAccel(I2C_HandleTypeDef *I2Cx, MPU6050_HandleTypeDef *mpu6050)
{
	HAL_StatusTypeDef retval;
	uint8_t tempBufferAccel[6];

	retval = HAL_I2C_Mem_Read(I2Cx, MPU6050_ADDR, ACCEL_XOUT_H_REG, 1, tempBufferAccel, 6, I2C_TIME_OUT);
	if(HAL_OK == retval)
	{
		int16_t accelRawX = (int16_t)(tempBufferAccel[0] << 8 | tempBufferAccel[1]);
		int16_t accelRawY = (int16_t)(tempBufferAccel[2] << 8 | tempBufferAccel[3]);
		int16_t accelRawZ = (int16_t)(tempBufferAccel[4] << 8 | tempBufferAccel[5]);
		accelRawX  -= ACCEL_X_OFFSET;
		accelRawY  -= ACCEL_Y_OFFSET;
		accelRawZ  -= ACCEL_Z_OFFSET;

		double accX  = accelRawX / ACCEL_CONFIG_SENS;
		double accY  = accelRawY / ACCEL_CONFIG_SENS;
		double accZ  = accelRawZ / ACCEL_CONFIG_SENS;


		mpu6050->accelAngleRoll = atan(accY / sqrt(accX * accX + accZ * accZ)) * RAD_TO_DEG;
		mpu6050->accelAnglePitch = -atan(accX / sqrt(accY * accY + accZ * accZ)) * RAD_TO_DEG; // TODO mpu arkasdınaki delikler drone'un arkası olacak. Burada kumandayla uyumlu olması için formülde "-" kaldırdık.
		return 0;
	}
	else
	{
		return 1;
	}

}

// sets gyroRates according to gyroscope values if reading process is done successfully
bool readGyro(I2C_HandleTypeDef *I2Cx, MPU6050_HandleTypeDef *mpu6050)
{
	HAL_StatusTypeDef retval;
	uint8_t tempBufferGyro[6];

	retval = HAL_I2C_Mem_Read(I2Cx, MPU6050_ADDR, GYRO_XOUT_H_REG, 1, tempBufferGyro, 6, I2C_TIME_OUT);
	if(HAL_OK == retval)
	{
		int16_t gyroRawX  = (int16_t)(tempBufferGyro[0] << 8 | tempBufferGyro[1]);
		int16_t gyroRawY  = (int16_t)(tempBufferGyro[2] << 8 | tempBufferGyro[3]);
		int16_t gyroRawZ  = (int16_t)(tempBufferGyro[4] << 8 | tempBufferGyro[5]);

		//Offset calibration of measurements.
		gyroRawX  -= GYRO_X_OFFSET;
		gyroRawY  -= GYRO_Y_OFFSET;
		gyroRawZ  -= GYRO_Z_OFFSET;

		mpu6050->gyroRateRoll   = gyroRawX / GYRO_CONFIG_SENS;
		mpu6050->gyroRatePitch  = gyroRawY / GYRO_CONFIG_SENS;
		mpu6050->gyroRateYaw    = gyroRawZ / GYRO_CONFIG_SENS;
		return 0; //No error
	}
	else
	{
		return 1;
	}

}
void setKalmanAngles(MPU6050_HandleTypeDef *mpu6050)
{
	// set Roll's
	mpu6050->kalmanAngleRoll = mpu6050->kalmanAngleRoll + mpu6050->deltaTime* mpu6050->gyroRateRoll;
	mpu6050->kalmanAngleUncertaintyRoll = mpu6050->kalmanAngleUncertaintyRoll + mpu6050->deltaTime * mpu6050->deltaTime * 16;
	double kalmanGainRoll = mpu6050->kalmanAngleUncertaintyRoll / (mpu6050->kalmanAngleUncertaintyRoll + 3*3);
	mpu6050->kalmanAngleRoll = mpu6050->kalmanAngleRoll + kalmanGainRoll *(mpu6050->accelAngleRoll - mpu6050->kalmanAngleRoll);
	mpu6050->kalmanAngleUncertaintyRoll *= (1 - kalmanGainRoll);

	// set Pitch's
	mpu6050->kalmanAnglePitch = mpu6050->kalmanAnglePitch + mpu6050->deltaTime* mpu6050->gyroRatePitch;
	mpu6050->kalmanAngleUncertaintyPitch= mpu6050->kalmanAngleUncertaintyPitch + mpu6050->deltaTime * 16;
	double kalmanGainPitch = mpu6050->kalmanAngleUncertaintyPitch / (mpu6050->kalmanAngleUncertaintyPitch + 3*3);
	mpu6050->kalmanAnglePitch = mpu6050->kalmanAnglePitch + kalmanGainPitch *(mpu6050->accelAnglePitch - mpu6050->kalmanAnglePitch);
	mpu6050->kalmanAngleUncertaintyPitch*= (1 - kalmanGainPitch);
	mpu6050->hasUnsafeAngle = hasUnsafeAngle(mpu6050->kalmanAngleRoll, mpu6050->kalmanAnglePitch);
}

// initialize kalman angles and uncertainty values

void readIMUAngleValues(I2C_HandleTypeDef* hi2c1, MPU6050_HandleTypeDef* mpu6050)
{
	uint32_t currentTime = HAL_GetTick();
	mpu6050->deltaTime =  (double)(currentTime - mpu6050->prevTime) / 1000.0; // We divided to 1000 to change ms to second.

	if(mpu6050->newDataAvailable)
	{

		mpu6050->hasError = readGyro(hi2c1, mpu6050)
							 || readAccel(hi2c1, mpu6050);
		setKalmanAngles(mpu6050);
		mpu6050->prevTime = currentTime;
	}

	if(mpu6050->deltaTime > 0.05 )
	{
		mpu6050->hasError = true;
	}

}
bool hasUnsafeAngle(double kalmanAngleRoll, double kalmanAnglePitch)
{
	bool isRollAngleUnsafety  = false;
	bool isPitchAngleUnsafety = false;
	if((kalmanAngleRoll > SAFETY_ANGLE || kalmanAngleRoll < -1*SAFETY_ANGLE))
	{
		isRollAngleUnsafety = true;
	}
	if((kalmanAnglePitch > SAFETY_ANGLE || kalmanAnglePitch < -1*SAFETY_ANGLE))
	{
		isPitchAngleUnsafety = true;
	}
	return isRollAngleUnsafety || isPitchAngleUnsafety;
}









