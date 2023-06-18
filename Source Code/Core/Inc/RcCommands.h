/*
 * rcCommand.h
 *
 *  Created on: Feb 6, 2023
 *      Author: esref
 */

#ifndef INC_RCCOMMANDS_H_
#define INC_RCCOMMANDS_H_

#include <stdbool.h>
#include <stdint.h>
#include "tim.h"
#include "pidController.h"

typedef struct
{
	bool isArmed;
	bool hasError;
	double throttle;
	double yawRateChange;
	double pitchAngle;
	double rollAngle;
}RC_HandleTypeDef;
void readRCAngleValues(RC_HandleTypeDef *RC_Commands);
bool RC_Commands_Init(TIM_HandleTypeDef *htim1, RC_HandleTypeDef *RC_Commands);
double mapDouble(double inValue, double inValueMin, double inValueMax, double outValueMin, double outValueMax);
bool rcCommandsArming(int CH7);
double mapAndLimit(double inValue, double inValueMin, double inValueMax, double outValueMin, double outValueMax);
uint32_t ignoreIntervalBand(int value, uint16_t TRANSMITTER_RAW_MIN, uint16_t TRANSMITTER_X_RAW_MAX);


#endif /* INC_RCCOMMANDS_H_ */
