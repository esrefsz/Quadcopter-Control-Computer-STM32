/*
 * tim.h
 *
 *  Created on: Feb 24, 2023
 *      Author: esref
 */

#ifndef INC_TIM_H_
#define INC_TIM_H_
#include "stm32g0xx_hal.h"
extern TIM_HandleTypeDef htim1;
extern TIM_HandleTypeDef htim2;

void MX_TIM1_Init(void);
void MX_TIM2_Init(void);



#endif /* INC_TIM_H_ */

