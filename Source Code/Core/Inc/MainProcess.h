/*
 * mainProcess.h
 *
 *  Created on: Feb 24, 2023
 *      Author: esref
 */
#ifndef INC_MAINPROCESS_H_
#define INC_MAINPROCESS_H_
//
#include "MotorPowers.h"
#include "PidController.h"
#include "Tim.h"

#define MPU_INTERRUPT_Pin GPIO_PIN_0
#define MPU_INTERRUPT_GPIO_Port GPIOC
#define MPU_INTERRUPT_EXTI_IRQn EXTI0_1_IRQn
#define BUZZER_Pin GPIO_PIN_1
#define BUZZER_GPIO_Port GPIOC
#define GREEN_LED_Pin GPIO_PIN_11
#define GREEN_LED_GPIO_Port GPIOA
#define RED_LED_Pin GPIO_PIN_12
#define RED_LED_GPIO_Port GPIOA
typedef struct
{
	MPU6050_HandleTypeDef MPU6050;
	RC_HandleTypeDef RC_Commands;
	Motor_PowersTypeDef Motor_Powers;
	Pid_AllHandleTypeDef Pid_All;

}FlightCCHandleTypeDef;

bool PWM_Output_Init(TIM_HandleTypeDef *htim2);
//void Error_Handler(void);
void MX_GPIO_Init();
void MX_IWDG_Init(void);
void mainProcessStart();
void SystemClock_Config(void);

#endif
