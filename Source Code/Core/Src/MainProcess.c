
#include "MainProcess.h"


FlightCCHandleTypeDef FlightCC;

IWDG_HandleTypeDef hiwdg;


void mainProcessStart() {

	HAL_Init();
	SystemClock_Config();
	MX_GPIO_Init();
	MX_I2C1_Init();
	MX_I2C2_Init();
	MX_TIM1_Init();
	MX_TIM2_Init();
	MX_IWDG_Init();
	HAL_GPIO_WritePin(GPIOA, RED_LED_Pin, GPIO_PIN_SET);
	for( int i = 0;i<3;i++)
	{
		HAL_GPIO_WritePin(BUZZER_GPIO_Port, BUZZER_Pin, GPIO_PIN_SET);
		HAL_Delay(10);
		HAL_GPIO_WritePin(BUZZER_GPIO_Port, BUZZER_Pin, GPIO_PIN_RESET);

	}

	resetPidAll(&(FlightCC.Pid_All));
	stopMotors(&(FlightCC.Motor_Powers)); // It is to be sure for safety.
	while (MPU6050_Init(&hi2c1, &(FlightCC.MPU6050)));
	while (RC_Commands_Init(&htim1, &(FlightCC.RC_Commands)));
	while (PWM_Output_Init(&htim2));


	//************************************
	//TODO kumanda arm olana kadar bir ışık yanıp sönmeli !!
	//TODO uçuşa hazır olduğunda bir saniye buzzera güç verip ışığı sürekli yanmalı.
	//while(rcCommandsArming(&htim1, &RC_Commands)); //TODO Burada arming olana kadar while dönügüsüne girmemesini sağlayabiliriz. Bunu düşünmemiz lazım.
	//Bu iki TODO interrupt ile yapılabilir.
	//************************************
	while (1)
	{

		readRCAngleValues(&(FlightCC.RC_Commands)); // TODO reciever safety için check edilmeli
		readIMUAngleValues(&hi2c1, &(FlightCC.MPU6050));

		if (FlightCC.MPU6050.hasError || FlightCC.RC_Commands.hasError )
		{

			resetPidAll(&(FlightCC.Pid_All));
			stopMotors(&(FlightCC.Motor_Powers));
			FlightCC.MPU6050.deltaTime = 0;
			HAL_GPIO_WritePin(GPIOA, RED_LED_Pin, GPIO_PIN_SET);
			HAL_GPIO_WritePin(GPIOA, GREEN_LED_Pin, GPIO_PIN_RESET);


		}
		else
		{
			HAL_GPIO_WritePin(GPIOA, RED_LED_Pin, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(GPIOA, GREEN_LED_Pin, GPIO_PIN_SET);


			HAL_IWDG_Refresh(&hiwdg);	// 1 Ms design decision.
			if (!FlightCC.RC_Commands.isArmed || FlightCC.RC_Commands.throttle < 10 || FlightCC.MPU6050.hasUnsafeAngle)
			{
				resetPidAll(&(FlightCC.Pid_All));
				stopMotors(&(FlightCC.Motor_Powers));
				continue;
			}

			if (FlightCC.MPU6050.newDataAvailable)
			{
				setMotorPowers(&(FlightCC.MPU6050), &(FlightCC.RC_Commands), &(FlightCC.Motor_Powers), &(FlightCC.Pid_All));
				FlightCC.MPU6050.newDataAvailable = false;
			}

		}


	}
}


void HAL_GPIO_EXTI_Rising_Callback(uint16_t GPIO_Pin)
{
  if (GPIO_Pin == MPU_INTERRUPT_Pin)
  {

	  FlightCC.MPU6050.newDataAvailable = true;
  }
}


void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(BUZZER_GPIO_Port, BUZZER_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GREEN_LED_Pin|RED_LED_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : MPU_INTERRUPT_Pin */
  GPIO_InitStruct.Pin = MPU_INTERRUPT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(MPU_INTERRUPT_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : BUZZER_Pin */
  GPIO_InitStruct.Pin = BUZZER_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(BUZZER_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : GREEN_LED_Pin RED_LED_Pin */
  GPIO_InitStruct.Pin = GREEN_LED_Pin|RED_LED_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI0_1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI0_1_IRQn);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}



void MX_IWDG_Init(void)
{

  /* USER CODE BEGIN IWDG_Init 0 */

  /* USER CODE END IWDG_Init 0 */

  /* USER CODE BEGIN IWDG_Init 1 */

  /* USER CODE END IWDG_Init 1 */
  hiwdg.Instance = IWDG;
  hiwdg.Init.Prescaler = IWDG_PRESCALER_256;
  hiwdg.Init.Window = 125;
  hiwdg.Init.Reload = 125;
  if (HAL_IWDG_Init(&hiwdg) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN IWDG_Init 2 */

  /* USER CODE END IWDG_Init 2 */

}
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI|RCC_OSCILLATORTYPE_LSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSIDiv = RCC_HSI_DIV1;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.LSIState = RCC_LSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = RCC_PLLM_DIV1;
  RCC_OscInitStruct.PLL.PLLN = 8;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}
