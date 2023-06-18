/*
 * pidController.h
 *
 *  Created on: Feb 6, 2023
 *      Author: esref
 */

#ifndef INC_PIDCONTROLLER_H_
#define INC_PIDCONTROLLER_H_

typedef struct
{
	double prevError;
	double prevIntegral;
}Pid_TypeDef;

typedef struct
{
	Pid_TypeDef pidAngleRollStruct;
	Pid_TypeDef pidAnglePitchStruct;
	Pid_TypeDef pidRateRollStruct;  //Struct mı yoksa global tanımlayıp extern ile ulaşmak mı? TODO
	Pid_TypeDef pidRatePitchStruct;
	Pid_TypeDef pidRateYawStruct;
}Pid_AllHandleTypeDef;



void resetPid(Pid_TypeDef *pidStruct);
double pidCalculation(double errorRate, double propootionalCoef, double integralCoef, double derivativeCoef, double deltaTime, Pid_TypeDef *pidSturct);
double limitValue(double compressedValue, double lowerLimit, double upperLimit);




#endif /* INC_PIDCONTROLLER_H_ */


