/*
 * i2c.h
 *
 *  Created on: Apr 29, 2023
 *      Author: esref
 */

#ifndef INC_I2C_H_
#define INC_I2C_H_
#include "stm32g0xx_hal.h"

extern I2C_HandleTypeDef hi2c1;
extern I2C_HandleTypeDef hi2c2;

void MX_I2C1_Init(void);
void MX_I2C2_Init(void);

void Error_Handler();


#endif /* INC_I2C_H_ */
