/*
 * MotorPowers.c
 *
 *  Created on: Apr 29, 2023
 *      Author: esref
 */


#include "MotorPowers.h"

#define PROPOTIONAL_ROLL_PITCH_ANGLE_COEF (2.0)
#define INTEGRAL_ROLL_PITCH_ANGLE_COEF	  (0.0)
#define DERIVATIVE_ROLL_PITCH_ANGLE_COEF  (0.0)

#define PROPOTIONAL_ROLL_PITCH_RATE_COEF  (0.6)
#define INTEGRAL_ROLL_PITCH_RATE_COEF	  (3.5)
#define DERIVATIVE_ROLL_PITCH_RATE_COEF   (0.03)

#define PROPOTIONAL_YAW_RATE_COEF (2)
#define INTEGRAL_YAW_RATE_COEF    (12)
#define DERIVATIVE_YAW_RATE_COEF  (0)

#define QUADCOPTER_MAX_ANGLE (20.00)
#define CONTROL_SIGNAL_YAW_LIMIT (180)
#define MOTOR_POWER_PWM_MIN_VALUE (1000)
#define MOTOR_POWER_PWM_MAX_VALUE (2000)

#define MOTOR_POWER_RATE_MIN_VALUE (0)
#define MOTOR_POWER_RATE_MAX_VALUE (180.0)

///******** KONTROL AMAÇLI***********
//double controlSignalRoll;
//double controlSignalPitch;
//double controlSignalYaw;

void setMotorPowers(MPU6050_HandleTypeDef *MPU6050, RC_HandleTypeDef *RC_Commands, Motor_PowersTypeDef *Motor_Powers, Pid_AllHandleTypeDef *Pid_All)
{


	  double errorAngleRoll  = RC_Commands->rollAngle - MPU6050->kalmanAngleRoll;
	  double errorAnglePitch = RC_Commands->pitchAngle - MPU6050->kalmanAnglePitch;

	  double desiredRateRoll  = pidCalculation(errorAngleRoll, PROPOTIONAL_ROLL_PITCH_ANGLE_COEF, INTEGRAL_ROLL_PITCH_ANGLE_COEF, DERIVATIVE_ROLL_PITCH_ANGLE_COEF, MPU6050->deltaTime, &(Pid_All->pidAngleRollStruct));
	  double desiredRatePitch = pidCalculation(errorAnglePitch,PROPOTIONAL_ROLL_PITCH_ANGLE_COEF, INTEGRAL_ROLL_PITCH_ANGLE_COEF, DERIVATIVE_ROLL_PITCH_ANGLE_COEF, MPU6050->deltaTime, &(Pid_All->pidAnglePitchStruct));

	  double errorRateRoll  = desiredRateRoll - MPU6050->gyroRateRoll;
	  double errorRatePitch = desiredRatePitch - MPU6050->gyroRatePitch;
	  double errorRateYaw   = RC_Commands->yawRateChange - MPU6050->gyroRateYaw; //TODO burası tam düzgün değil.

	  double controlSignalRoll  = pidCalculation(errorRateRoll, PROPOTIONAL_ROLL_PITCH_RATE_COEF, INTEGRAL_ROLL_PITCH_RATE_COEF, DERIVATIVE_ROLL_PITCH_RATE_COEF, MPU6050->deltaTime, &(Pid_All->pidRateRollStruct));
	  double controlSignalPitch = pidCalculation(errorRatePitch,PROPOTIONAL_ROLL_PITCH_RATE_COEF, INTEGRAL_ROLL_PITCH_RATE_COEF, DERIVATIVE_ROLL_PITCH_RATE_COEF, MPU6050->deltaTime, &(Pid_All->pidRatePitchStruct));
	  double controlSignalYaw   = pidCalculation(errorRateYaw,  PROPOTIONAL_YAW_RATE_COEF, INTEGRAL_YAW_RATE_COEF, DERIVATIVE_YAW_RATE_COEF, MPU6050->deltaTime, &(Pid_All->pidRateYawStruct));

	  double controlFrontLeft     = -1*controlSignalRoll - controlSignalPitch + controlSignalYaw;
	  double controlFrontRight    = controlSignalRoll - controlSignalPitch - controlSignalYaw;
	  double controlRearLeft      = -1*controlSignalRoll + controlSignalPitch - controlSignalYaw;
	  double controlRearRight     = controlSignalRoll + controlSignalPitch + controlSignalYaw;

	  double throttlePWM = mapDouble(RC_Commands->throttle, MOTOR_POWER_RATE_MIN_VALUE, MOTOR_POWER_RATE_MAX_VALUE, MOTOR_POWER_PWM_MIN_VALUE, MOTOR_POWER_PWM_MAX_VALUE);
	  Motor_Powers->pwmFrontLeftMotorPower  = throttlePWM + controlFrontLeft;
	  Motor_Powers->pwmFrontRightMotorPower = throttlePWM + controlFrontRight;
	  Motor_Powers->pwmRearLeftMotorPower   = throttlePWM + controlRearLeft;
	  Motor_Powers->pwmRearRightMotorPower  = throttlePWM + controlRearRight;


	  reduceAndLimitMotorPowers(Motor_Powers);
	  setEscSpeed(&htim2 , *Motor_Powers);
}
void setEscSpeed(TIM_HandleTypeDef *htim3, Motor_PowersTypeDef Motor_Powers)
{

  __HAL_TIM_SET_COMPARE(htim3, TIM_CHANNEL_1, Motor_Powers.pwmFrontLeftMotorPower);
  __HAL_TIM_SET_COMPARE(htim3, TIM_CHANNEL_2, Motor_Powers.pwmFrontRightMotorPower);
  __HAL_TIM_SET_COMPARE(htim3, TIM_CHANNEL_3, Motor_Powers.pwmRearLeftMotorPower);
  __HAL_TIM_SET_COMPARE(htim3, TIM_CHANNEL_4, Motor_Powers.pwmRearRightMotorPower);
}

void resetPidAll(Pid_AllHandleTypeDef *Pid_All)
{
	resetPid(&(Pid_All->pidRateRollStruct));
	resetPid(&(Pid_All->pidRatePitchStruct));
	resetPid(&(Pid_All->pidRateYawStruct));

	resetPid(&(Pid_All->pidAngleRollStruct));
	resetPid(&(Pid_All->pidAnglePitchStruct));
}

void stopMotors(Motor_PowersTypeDef *Motor_Powers)
{

	Motor_Powers->pwmFrontLeftMotorPower  = MOTOR_POWER_PWM_MIN_VALUE;
    Motor_Powers->pwmFrontRightMotorPower = MOTOR_POWER_PWM_MIN_VALUE;
    Motor_Powers->pwmRearLeftMotorPower   = MOTOR_POWER_PWM_MIN_VALUE;
    Motor_Powers->pwmRearRightMotorPower  = MOTOR_POWER_PWM_MIN_VALUE;
	setEscSpeed(&htim2 , *Motor_Powers);

}

double maxValue(double first, double second, double third, double fourth)
{
	int max = first;
	if(second > max) max = second;
	if(third > max) max = third;
	if(fourth > max) max = fourth;
	return max;
}
void reduceAndLimitMotorPowers(Motor_PowersTypeDef *Motor_Powers)
{
	  double maxValueOfMotors = maxValue(Motor_Powers->pwmFrontLeftMotorPower, Motor_Powers->pwmFrontRightMotorPower, Motor_Powers->pwmRearLeftMotorPower, Motor_Powers->pwmRearRightMotorPower);
	  if (maxValueOfMotors > MOTOR_POWER_PWM_MAX_VALUE)
	  {
		  double motorPowersRate = maxValueOfMotors / MOTOR_POWER_PWM_MAX_VALUE;
		  Motor_Powers->pwmFrontLeftMotorPower   /= motorPowersRate;
		  Motor_Powers->pwmFrontRightMotorPower  /= motorPowersRate;
		  Motor_Powers->pwmRearLeftMotorPower    /= motorPowersRate;
		  Motor_Powers->pwmRearRightMotorPower   /= motorPowersRate;

	  }

	  Motor_Powers->pwmFrontLeftMotorPower  = limitValue(Motor_Powers->pwmFrontLeftMotorPower,  MOTOR_POWER_PWM_MIN_VALUE, MOTOR_POWER_PWM_MAX_VALUE);
      Motor_Powers->pwmFrontRightMotorPower = limitValue(Motor_Powers->pwmFrontRightMotorPower, MOTOR_POWER_PWM_MIN_VALUE, MOTOR_POWER_PWM_MAX_VALUE);
	  Motor_Powers->pwmRearLeftMotorPower   = limitValue(Motor_Powers->pwmRearLeftMotorPower,   MOTOR_POWER_PWM_MIN_VALUE, MOTOR_POWER_PWM_MAX_VALUE);
	  Motor_Powers->pwmRearRightMotorPower  = limitValue(Motor_Powers->pwmRearRightMotorPower,  MOTOR_POWER_PWM_MIN_VALUE, MOTOR_POWER_PWM_MAX_VALUE);

}


bool PWM_Output_Init(TIM_HandleTypeDef *htim2)
{
	  HAL_StatusTypeDef retval;
	  retval =  HAL_TIM_PWM_Start(htim2, TIM_CHANNEL_1); // TIMX->DIER, CCXIE bitini aktif eder.
	  retval |= HAL_TIM_PWM_Start(htim2, TIM_CHANNEL_2);
	  retval |= HAL_TIM_PWM_Start(htim2, TIM_CHANNEL_3);
	  retval |= HAL_TIM_PWM_Start(htim2, TIM_CHANNEL_4);
	  if (HAL_OK == retval)
	  {
		  return 0;
	  }
	  else
	  {
		  return 1;
	  }
}
