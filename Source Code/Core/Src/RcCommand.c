/*
 * RcCommand.c
 *
 *  Created on: Apr 29, 2023
 *      Author: esref
 */



#include "RcCommands.h"
#include <stdlib.h> // This is for abs;
// TODO kumanda konfigurasyonyla ilgili bir fonksiyon yazılmalı.
#define TRANSMITTER_THROTTLE_RAW_MIN (1000) // Mesaured.
#define TRANSMITTER_THROTTLE_RAW_MAX (2000) // Mesaured.
#define TRANSMITTER_INTERVAL_BAND_RAW (50)

// Mean Value 1475.
#define TRANSMITTER_YAW_RAW_MIN      (1000) // Mesaured.
#define TRANSMITTER_YAW_RAW_MAX      (2000) // Mesaured.

//Mean Value 1500.
#define TRANSMITTER_PITCH_RAW_MIN    (1000)  // Measured.
#define TRANSMITTER_PITCH_RAW_MAX    (2000) // Measured.

// Mean Value 1481.
#define TRANSMITTER_ROLL_RAW_MIN     (1000) // Measured.
#define TRANSMITTER_ROLL_RAW_MAX     (2000) // Measured.

#define THROTTLE_ANGLE_MIN           (0)
#define THROTTLE_ANGLE_MAX           (180) //This is design decision.
#define ROLL_ANGLE_MIN               (-20) //This is design decision.
#define ROLL_ANGLE_MAX               (20)  //This is design decision.
#define PITCH_ANGLE_MIN              (-20) //This is design decision.
#define PITCH_ANGLE_MAX              (20)  //This is design decision.
#define YAW_RATE_MIN                 (-180)//This is design decision.
#define YAW_RATE_MAX                 (180) //This is design decision.
#define ARM_THRESHOLD 			     (1500)
//volatile uint32_t ch1_rising = 0, ch2_rising = 0, ch3_rising = 0, ch4_rising = 0;
//volatile uint32_t ch1_falling = 0, ch2_falling = 0, ch3_falling = 0, ch4_falling = 0;
//volatile uint32_t pre_ch1 = 0, ch1 = 0, pre_ch2 = 0, ch2 = 0, pre_ch3 = 0, ch3 = 0, pre_ch4 = 0, ch4 = 0;
//void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim)
//{
//	if(htim->Instance == TIM1)  // interrupt TIM1 biriminden geliyorsa gir
//	{
//		switch(htim->Channel) // aktif kanal hamgisiyse, o kanalın case'ine git
//		{
//		case HAL_TIM_ACTIVE_CHANNEL_1:
//			if((TIM1->CCER & TIM_CCER_CC1P)==0) // kanalin aktif olmasi kesmenin oradan gelecegi anlamina gelmez/gpio pinini kontrol et
//			{
//				ch1_rising = TIM1->CCR1; // yukselen kenar degerini kaydet
//				TIM1->CCER |= TIM_CCER_CC1P; // polariteyi düsen kenar olarak degistir
//			}
//
//			else
//			{
//				ch1_falling = TIM1->CCR1;
//				pre_ch1 = ch1_falling - ch1_rising; // dusen kenar degerini kaydet ve yukselen kenar degerinden cikar
//				if(pre_ch1 < 0)pre_ch1 += 0xFFFF;// eger sonuc negatifse taban tumleme yap
//				if(pre_ch1 < 2010 && pre_ch1 > 990)ch1=pre_ch1;
//				TIM1->CCER &= ~TIM_CCER_CC1P; // polariteyi yukselen kenar olarak ayarla
//
//				/*
//				 * ch1_rising 65000
//				 * ch1 falling 570
//				 * pre_ch1 = pre_ch1 falling - pre_ch1_rising = 570 - 65000 = -64430
//				 * pre_ch1 +=0xFFFF(65536) --> 1106
//				 */
//			}
//			break;
//		case HAL_TIM_ACTIVE_CHANNEL_2:
//			if((TIM1->CCER & TIM_CCER_CC2P)==0)
//			{
//				ch2_rising = TIM1->CCR2;
//				TIM1->CCER |= TIM_CCER_CC2P;
//			}
//			else
//			{
//				ch2_falling = TIM1->CCR2;
//				pre_ch2 = ch2_falling - ch2_rising;
//				if(pre_ch2 < 0)pre_ch2 += 0xFFFF;
//				if(pre_ch2 < 2010 && pre_ch2 > 990)ch2=pre_ch2;
//				TIM1->CCER &= ~TIM_CCER_CC2P;
//			}
//			break;
//		case HAL_TIM_ACTIVE_CHANNEL_3:
//			if((TIM1->CCER & TIM_CCER_CC3P)==0)
//			{
//				ch3_rising = TIM1->CCR3;
//				TIM1->CCER |= TIM_CCER_CC3P;
//			}
//			else
//			{
//				ch3_falling = TIM1->CCR3;
//				pre_ch3 = ch3_falling - ch3_rising;
//				if(pre_ch3 < 0)pre_ch3 += 0xFFFF;
//				if(pre_ch3 < 2010 && pre_ch3 > 990)ch3=pre_ch3;
//				TIM1->CCER &= ~TIM_CCER_CC3P;
//			}
//			break;
//		case HAL_TIM_ACTIVE_CHANNEL_4:
//			if((TIM1->CCER & TIM_CCER_CC4P)==0)
//			{
//				ch4_rising = TIM1->CCR4;
//				TIM1->CCER |= TIM_CCER_CC4P;
//			}
//			else
//			{
//				ch4_falling = TIM1->CCR4;
//				pre_ch4 = ch4_falling - ch4_rising;
//				if(pre_ch4 < 0)pre_ch4 += 0xFFFF;
//				if(pre_ch4 < 2010 && pre_ch4 > 990)ch4=pre_ch4;
//				TIM1->CCER &= ~TIM_CCER_CC4P;
//			}
//			break;
//		default:
//			break;
//		}
//	}
//}
//volatile int i=0,cntrl =0;
//volatile uint16_t ch[9], ch1=0, ch2=0, ch3=0, ch4=0, ch5=0, ch6=0, ch7=0, ch8=0;
//volatile uint16_t prevtime,current_time,elapsedtime;
//volatile int a =0;

//void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim)
//{
//	if(htim->Instance == TIM1)  // interrupt TIM1 biriminden geliyorsa gir
//		{
//			if(htim->Channel == HAL_TIM_ACTIVE_CHANNEL_1) //interrupt kanal 1'den geliyorsa gir
//			{
//
//				if((TIM1->CCER & TIM_CCER_CC1P)==0) // interrupt yükselen kenar ile olustuysa gir
//				{
//					if(cntrl == 1)
//					{
//						a++;
//						i++;
//						switch(i) // i degerine göre case git
//						{
//						case 1:
//							ch[0] = TIM1->CCR1;
//							break;
//						case 2:
//							ch[1] = TIM1->CCR1;
//							ch1 = ch[1]-ch[0];
//							if (ch1<0)ch1+=0xFFFF;
//							break;
//
//							/*
//							 * ch[0] 65000
//							 * ch[1] 570
//							 * ch1 = ch[1] - ch[0] = 570 - 65000 = -64430
//							 * ch1 +=0xFFFF(65535) --> 1105
//							 */
//						case 3:
//							ch[2] = TIM1->CCR1;
//							ch2 = ch[2]-ch[1];
//							if (ch2<0)ch2+=0xFFFF;
//							break;
//						case 4:
//							ch[3] = TIM1->CCR1;
//							ch3 = ch[3]-ch[2];
//							if (ch3<0)ch3+=0xFFFF;
//							break;
//						case 5:
//							ch[4] = TIM1->CCR1;
//							ch4 = ch[4]-ch[3];
//							if (ch4<0)ch4+=0xFFFF;
//							break;
//						case 6:
//							ch[5] = TIM1->CCR1;
//							ch5 = ch[5]-ch[4];
//							if (ch5<0)ch5+=0xFFFF;
//							break;
//						case 7:
//							ch[6] = TIM1->CCR1;
//							ch6 = ch[6]-ch[5];
//							if (ch6<0)ch6+=0xFFFF;
//							break;
//						case 8:
//							ch[7] = TIM1->CCR1;
//							ch7 = ch[7]-ch[6];
//							if (ch7<0)ch7+=0xFFFF;
//							i=0;
//
//							break;
//						case 9:
//							ch[8] = TIM1->CCR1;
//							ch8 = ch[8]-ch[7];
//							if (ch8<0)ch8+=0xFFFF;
//							break;
//						default:
//							break;
//					}
//
//					}
//					else
//					{
//						current_time = HAL_GetTick();
//						elapsedtime=(current_time-prevtime)*1000;
//						prevtime = current_time;
//						if(elapsedtime>2010) // elapsedtime >= 2010
//						{
//							ch[0] = TIM1->CCR1;
//							cntrl=1;
//							i=1;
//						}
//					}
//				}
//			}
//		}
//}
uint16_t counter[9], ch[8], preCH[8], index = 0, currentTime = 0, prevTime = 0, elapsedTime=0;
void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim)
{
	if(htim->Instance == TIM1)  // interrupt TIM1 biriminden geliyorsa gir
		{
			if(htim->Channel == HAL_TIM_ACTIVE_CHANNEL_1) //interrupt kanal 1'den geliyorsa gir
			{
				if((TIM1->CCER & TIM_CCER_CC1P)==0) // interrupt yükselen kenar ile olustuysa gir
				{
					if(0 == index)
					{
							counter[index] = TIM1->CCR1;
					}
					else
					{
						counter[index] = TIM1->CCR1;
						ch[index-1] = counter[index] - counter[index-1];
						if(ch[index-1]<0)  ch[index-1] += 0xFFFF;
						if(ch[index-1]>2010 || ch[index-1] <990)
						{
							ch[index-1] = preCH[index-1];
						}
						else
						{
							preCH[index-1] = ch[index-1];
						}
					}

					index++;
					currentTime = HAL_GetTick();
					elapsedTime=(currentTime-prevTime)*1000;
					prevTime = currentTime;

					if (elapsedTime>2010)
					{
						index = 1;
						counter[0] = TIM1->CCR1;
					}



				}

				}
		}
}

bool RC_Commands_Init(TIM_HandleTypeDef *htim1, RC_HandleTypeDef *RC_Commands)
{
	  HAL_StatusTypeDef retval;
	  RC_Commands->isArmed = false; //That must start with the case which is not armed.
	  retval =  HAL_TIM_IC_Start_IT(htim1, TIM_CHANNEL_1); // TIMX->DIER, CCXIE bitini aktif eder.
//	  retval |= HAL_TIM_IC_Start_IT(htim1, TIM_CHANNEL_2);
//	  retval |= HAL_TIM_IC_Start_IT(htim1, TIM_CHANNEL_3);
//	  retval |= HAL_TIM_IC_Start_IT(htim1, TIM_CHANNEL_4);
	  if (HAL_OK == retval)
	  {
		  RC_Commands->hasError = false;
		  return 0;
	  }
	  else
	  {
		  RC_Commands->hasError = true;

		  return 1;
	  }
}


uint32_t ignoreIntervalBand(int value, uint16_t TRANSMITTER_RAW_MIN, uint16_t TRANSMITTER_X_RAW_MAX)
{
	int center = (TRANSMITTER_RAW_MIN + TRANSMITTER_X_RAW_MAX) / 2;
	if (abs(value - center) <= TRANSMITTER_INTERVAL_BAND_RAW)
	    return center;
	  else
	    return value;
}
void readRCAngleValues(RC_HandleTypeDef *RC_Commands)
{
	RC_Commands->isArmed = rcCommandsArming(ch[6]);
	ch[0] = ignoreIntervalBand(ch[0], TRANSMITTER_ROLL_RAW_MIN, TRANSMITTER_ROLL_RAW_MAX);
	ch[1] = ignoreIntervalBand(ch[1], TRANSMITTER_PITCH_RAW_MIN, TRANSMITTER_PITCH_RAW_MAX);
	ch[3] = ignoreIntervalBand(ch[3], TRANSMITTER_YAW_RAW_MIN, TRANSMITTER_YAW_RAW_MAX);
	// TODO burada kumanda bozuk olduğu için bir kısıtlama getirmiştik. Generic hale getirilmesi lazım.
	RC_Commands->rollAngle     = mapAndLimit(ch[0], TRANSMITTER_ROLL_RAW_MIN,     TRANSMITTER_ROLL_RAW_MAX,     ROLL_ANGLE_MIN,     ROLL_ANGLE_MAX);
	RC_Commands->pitchAngle    = mapAndLimit(ch[1], TRANSMITTER_PITCH_RAW_MIN,    TRANSMITTER_PITCH_RAW_MAX,    PITCH_ANGLE_MIN,    PITCH_ANGLE_MAX);
	RC_Commands->throttle      = mapAndLimit(ch[2], TRANSMITTER_THROTTLE_RAW_MIN, TRANSMITTER_THROTTLE_RAW_MAX, THROTTLE_ANGLE_MIN, THROTTLE_ANGLE_MAX);
	RC_Commands->yawRateChange = mapAndLimit(ch[3], TRANSMITTER_YAW_RAW_MIN,      TRANSMITTER_YAW_RAW_MAX,      YAW_RATE_MIN,       YAW_RATE_MAX);
}

double mapDouble(double inValue, double inValueMin, double inValueMax, double outValueMin, double outValueMax)
{
	return (inValue - inValueMin) * (outValueMax - outValueMin) / (inValueMax - inValueMin) + outValueMin;
}

double mapAndLimit(double inValue, double inValueMin, double inValueMax, double outValueMin, double outValueMax)
{
	inValue = limitValue(inValue, inValueMin, inValueMax); //Firstly it limited to their offset values.
	return mapDouble(inValue, inValueMin, inValueMax, outValueMin, outValueMax);// Then it occures the value in output interval.
}
bool rcCommandsArming(int CH7)
{


	if(CH7 < ARM_THRESHOLD)
	{
		return false;
	}
	else
	{

		return true;
	}
}
