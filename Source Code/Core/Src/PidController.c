/*
 * pidController.c
 *
 *  Created on: Feb 6, 2023
 *      Author: esref
 */


#include "pidController.h"
#define TIME_ITERATION (0.004)
#define PID_LIMIT (400) // TODO tam mantığı ne anlayamadım?

double pidCalculation(double errorRate, double propootionalCoef, double integralCoef, double derivativeCoef, double deltaTime, Pid_TypeDef *pidSturct)
{
	double currentPropotionalTerm = propootionalCoef * errorRate;
	double currentIntegralTerm    = pidSturct->prevIntegral + integralCoef * (errorRate + pidSturct->prevError)/2 * deltaTime;
	double currentDerivativeTerm  = derivativeCoef * (errorRate - pidSturct->prevError) / deltaTime;

	currentIntegralTerm = limitValue(currentIntegralTerm, -1*PID_LIMIT, PID_LIMIT);

	double controlSignalOutput = currentPropotionalTerm + currentIntegralTerm + currentDerivativeTerm;
	controlSignalOutput = limitValue(controlSignalOutput, -1*PID_LIMIT, PID_LIMIT);
	pidSturct->prevError = errorRate;
	pidSturct->prevIntegral = currentIntegralTerm;
	return controlSignalOutput;
}
double limitValue(double compressedValue, double lowerLimit, double upperLimit)
{
	if ((lowerLimit < compressedValue) && (compressedValue < upperLimit))
	{
		return compressedValue;
	}
	else if(lowerLimit >= compressedValue)
	{
		return lowerLimit;
	}
	else
	{
		return upperLimit;
	}
}
void resetPid(Pid_TypeDef *pidStruct)
{
	pidStruct->prevError = 0;
	pidStruct->prevIntegral = 0;
}




