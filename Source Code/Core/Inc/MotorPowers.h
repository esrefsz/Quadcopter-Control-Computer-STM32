/*
 * MotorPowers.h
 *
 *  Created on: Apr 29, 2023
 *      Author: esref
 */

#include "RcCommands.h"
#include "MPU6050.h" //TODO Bu sadece struct decleration için tanımlandı. Struct lardan oluşan headerlar mı tanımlasak ?


typedef struct
{
	uint16_t pwmFrontLeftMotorPower;  // 1000-2000.
	uint16_t pwmFrontRightMotorPower; // 1000-2000.
	uint16_t pwmRearLeftMotorPower;   // 1000-2000.
	uint16_t pwmRearRightMotorPower;  // 1000-2000.
}Motor_PowersTypeDef;

void setMotorPowers(MPU6050_HandleTypeDef *MPU6050, RC_HandleTypeDef *RC_Commands, Motor_PowersTypeDef *Motor_Powers, Pid_AllHandleTypeDef *Pid_All);
void stopMotors(Motor_PowersTypeDef *Motor_Powers);
void resetPidAll(Pid_AllHandleTypeDef *Pid_All);
void reduceAndLimitMotorPowers(Motor_PowersTypeDef *Motor_Powers);
double maxValue(double first, double second, double third, double fourth);
void setEscSpeed(TIM_HandleTypeDef *htim2, Motor_PowersTypeDef Motor_Powers);
