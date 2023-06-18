/*
 * mpu6050.h
 *
 *  Created on: Feb 6, 2023
 *      Author: esref
 */


#ifndef INC_MPU6050_H_
#define INC_MPU6050_H_

#include <math.h>
#include <stdint.h>
#include <stdbool.h>
#include "I2c.h"

typedef struct
{
	volatile bool newDataAvailable;
	bool hasError;
	bool hasUnsafeAngle;
	uint32_t prevTime; // TODO Bunun type kontrol etmemiz lazım. Sıfırlandığında neler olacağını.
	double deltaTime;
	float temperature;

	double gyroRateRoll;
	double gyroRatePitch;
	double gyroRateYaw;

    double accelAngleRoll;
    double accelAnglePitch;

    double kalmanAngleRoll;
    double kalmanAnglePitch;
    double kalmanAngleUncertaintyRoll;
    double kalmanAngleUncertaintyPitch;

} MPU6050_HandleTypeDef;

bool hasUnsafeAngle(double kalmanAngleRoll, double kalmanAnglePitch);
bool MPU6050_Init(I2C_HandleTypeDef *I2C, MPU6050_HandleTypeDef *mpu6050);

bool readAccel(I2C_HandleTypeDef *I2Cx, MPU6050_HandleTypeDef *mpu6050);

bool readGyro(I2C_HandleTypeDef *I2Cx, MPU6050_HandleTypeDef *mpu6050);

bool readTemp(I2C_HandleTypeDef *I2Cx, MPU6050_HandleTypeDef *mpu6050);

void setKalmanAngles(MPU6050_HandleTypeDef *mpu6050);

void readIMUAngleValues(I2C_HandleTypeDef *I2Cx, MPU6050_HandleTypeDef *mpu6050);


#endif /* INC_MPU6050_H_ */

